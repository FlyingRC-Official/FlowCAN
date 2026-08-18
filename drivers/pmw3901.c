#include "pmw3901.h"
#include "config.h"
#include "platform.h"
#include <stddef.h>

typedef struct { uint8_t reg; uint8_t value; uint16_t delay_ms; } reg_init_t;

/* MIT sequence adapted from Bitcraze_PMW3901 commit d322d98d6f61757352d11d922cd194539e165231. */
static const reg_init_t init_sequence[] = {
    {0x7F,0x00,0},{0x61,0xAD,0},{0x7F,0x03,0},{0x40,0x00,0},{0x7F,0x05,0},{0x41,0xB3,0},{0x43,0xF1,0},{0x45,0x14,0},{0x5B,0x32,0},{0x5F,0x34,0},{0x7B,0x08,0},
    {0x7F,0x06,0},{0x44,0x1B,0},{0x40,0xBF,0},{0x4E,0x3F,0},{0x7F,0x08,0},{0x65,0x20,0},{0x6A,0x18,0},{0x7F,0x09,0},{0x4F,0xAF,0},{0x5F,0x40,0},
    {0x48,0x80,0},{0x49,0x80,0},{0x57,0x77,0},{0x60,0x78,0},{0x61,0x78,0},{0x62,0x08,0},{0x63,0x50,0},{0x7F,0x0A,0},{0x45,0x60,0},{0x7F,0x00,0},
    {0x4D,0x11,0},{0x55,0x80,0},{0x74,0x1F,0},{0x75,0x1F,0},{0x4A,0x78,0},{0x4B,0x78,0},{0x44,0x08,0},{0x45,0x50,0},{0x64,0xFF,0},{0x65,0x1F,0},
    {0x7F,0x14,0},{0x65,0x60,0},{0x66,0x08,0},{0x63,0x78,0},{0x7F,0x15,0},{0x48,0x58,0},{0x7F,0x07,0},{0x41,0x0D,0},{0x43,0x14,0},{0x4B,0x0E,0},
    {0x45,0x0F,0},{0x44,0x42,0},{0x4C,0x80,0},{0x7F,0x10,0},{0x5B,0x02,0},{0x7F,0x07,0},{0x40,0x41,0},{0x70,0x00,100},{0x32,0x44,0},{0x7F,0x07,0},
    {0x40,0x40,0},{0x7F,0x06,0},{0x62,0xF0,0},{0x63,0x00,0},{0x7F,0x0D,0},{0x48,0xC0,0},{0x6F,0xD5,0},{0x7F,0x00,0},{0x5B,0xA0,0},{0x4E,0xA8,0},{0x5A,0x50,0},{0x40,0x80,0}
};

static void write_reg(uint8_t reg, uint8_t value)
{
    platform_pmw_cs(true); platform_delay_us(50U); platform_spi_transfer((uint8_t)(reg | 0x80U)); platform_spi_transfer(value); platform_delay_us(50U); platform_pmw_cs(false); platform_delay_us(200U);
}
static uint8_t read_reg(uint8_t reg)
{
    platform_pmw_cs(true); platform_delay_us(50U); platform_spi_transfer((uint8_t)(reg & 0x7FU)); platform_delay_us(50U); const uint8_t v = platform_spi_transfer(0U); platform_delay_us(100U); platform_pmw_cs(false); return v;
}

enum { INIT_IDLE, INIT_RESET_ASSERTED, INIT_RELEASE_WAIT, INIT_ID_WAIT, INIT_SEQUENCE, INIT_DONE, INIT_FAILED };

void pmw3901_reset_state(pmw3901_t *d, uint32_t now) { d->accumulated_x=0; d->accumulated_y=0; d->integration_start_us=now; d->quality=0; d->initialized=false; d->init_state=INIT_IDLE; d->init_index=0U; d->init_deadline_us=now; }

void pmw3901_start_init(pmw3901_t *d, uint32_t now)
{
    d->initialized=false;d->accumulated_x=0;d->accumulated_y=0;d->quality=0;d->integration_start_us=now;d->init_index=0U;d->init_state=INIT_RESET_ASSERTED;d->init_deadline_us=now+1000U;platform_pmw_reset(true);
}

pmw_init_result_t pmw3901_init_step(pmw3901_t *d, uint32_t now)
{
    if((int32_t)(now-d->init_deadline_us)<0)return PMW_INIT_BUSY;
    if(d->init_state==INIT_RESET_ASSERTED){platform_pmw_reset(false);d->init_state=INIT_RELEASE_WAIT;d->init_deadline_us=now+5000U;return PMW_INIT_BUSY;}
    if(d->init_state==INIT_RELEASE_WAIT){write_reg(0x3A,0x5A);d->init_state=INIT_ID_WAIT;d->init_deadline_us=now+5000U;return PMW_INIT_BUSY;}
    if(d->init_state==INIT_ID_WAIT){if(read_reg(0x00)!=PMW3901_PRODUCT_ID||read_reg(0x5F)!=PMW3901_INVERSE_ID){d->init_state=INIT_FAILED;return PMW_INIT_FAULT;}for(uint8_t r=0x02;r<=0x06;r++)(void)read_reg(r);d->init_state=INIT_SEQUENCE;}
    if(d->init_state==INIT_SEQUENCE){const size_t count=sizeof(init_sequence)/sizeof(init_sequence[0]);if(d->init_index<count){const reg_init_t *entry=&init_sequence[d->init_index++];write_reg(entry->reg,entry->value);d->init_deadline_us=now+(uint32_t)entry->delay_ms*1000U;return PMW_INIT_BUSY;}d->initialized=true;d->integration_start_us=now;d->init_state=INIT_DONE;return PMW_INIT_READY;}
    return d->init_state==INIT_DONE?PMW_INIT_READY:PMW_INIT_FAULT;
}

bool pmw3901_read_motion(pmw_motion_t *m)
{
    uint8_t b[12];
    platform_pmw_cs(true); platform_delay_us(35U); platform_spi_transfer(0x16U);
    platform_delay_us(35U); for (size_t i=0;i<sizeof(b);i++) b[i]=platform_spi_transfer(0U); platform_delay_us(100U); platform_pmw_cs(false);
    m->motion=(b[0]&0x80U)!=0U; m->quality=b[6]; m->dx=(int16_t)((uint16_t)b[3]<<8U|b[2]); m->dy=(int16_t)((uint16_t)b[5]<<8U|b[4]); return true;
}

void pmw3901_transform(int16_t rx, int16_t ry, int32_t *x, int32_t *y)
{
    int32_t a=rx,b=ry;
#if FLOW_SWAP_XY
    int32_t t=a;a=b;b=t;
#endif
#if FLOW_ROTATION_DEG == 90
    int32_t t=a;a=-b;b=t;
#elif FLOW_ROTATION_DEG == 180
    a=-a;b=-b;
#elif FLOW_ROTATION_DEG == 270
    int32_t t=a;a=b;b=-t;
#endif
    *x=a*FLOW_SIGN_X; *y=b*FLOW_SIGN_Y;
}

void pmw3901_accumulate(pmw3901_t *d, const pmw_motion_t *m)
{
    if (!m->motion) return;
    int32_t x,y;
    pmw3901_transform(m->dx,m->dy,&x,&y);
    int64_t sx=(int64_t)d->accumulated_x+x,sy=(int64_t)d->accumulated_y+y;
    d->accumulated_x=sx>INT32_MAX?INT32_MAX:(sx<INT32_MIN?INT32_MIN:(int32_t)sx);
    d->accumulated_y=sy>INT32_MAX?INT32_MAX:(sy<INT32_MIN?INT32_MIN:(int32_t)sy);
    d->quality=m->quality;
}

void pmw3901_publish(pmw3901_t *d, uint32_t now, flow_sample_t *s)
{
    s->timestamp_us=now; s->integration_us=now-d->integration_start_us; s->count_x=d->accumulated_x; s->count_y=d->accumulated_y;
    s->integral_x_rad=(float)d->accumulated_x*FLOW_SCALE_X*FLOW_RAD_PER_COUNT; s->integral_y_rad=(float)d->accumulated_y*FLOW_SCALE_Y*FLOW_RAD_PER_COUNT;
    s->quality=d->quality; s->valid=d->initialized; d->accumulated_x=0; d->accumulated_y=0; d->quality=0; d->integration_start_us=now;
}
