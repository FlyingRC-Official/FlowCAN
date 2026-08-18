#include "vl53l1x.h"
#include "config.h"
#include "flowcan/time_utils.h"
#include "platform.h"
#include "VL53L1X_api.h"

extern const uint8_t VL51L1X_DEFAULT_CONFIGURATION[];
#define TOF_CONFIG_FIRST 0x2DU
#define TOF_CONFIG_LAST 0x87U
#define TOF_BOOT_TIMEOUT_MS 500U

static void invalidate(range_sample_t *s, uint32_t now_us, range_status_t status)
{
    s->timestamp_us=now_us; s->distance_mm=-1; s->quality=0; s->status=status;
}

void tof_begin(tof_t *t, uint32_t now)
{
    platform_tof_xshut(false); t->state=TOF_BOOT_WAIT; t->deadline_ms=now+2U; t->timeout_ms=0U; t->init_index=0U; t->irq_pending=false;
}

void tof_irq_notify(tof_t *t) { t->irq_pending=true; }

static bool configure_continuous(void)
{
    return VL53L1X_SetDistanceMode(VL53L1X_I2C_ADDRESS,2U)==0 &&
           VL53L1X_SetTimingBudgetInMs(VL53L1X_I2C_ADDRESS,VL53L1X_TIMING_BUDGET_MS)==0 &&
           VL53L1X_SetInterMeasurementInMs(VL53L1X_I2C_ADDRESS,VL53L1X_INTER_MEASUREMENT_MS)==0 &&
           VL53L1X_StartRanging(VL53L1X_I2C_ADDRESS)==0;
}

static void enter_fault(tof_t *t,uint32_t now,range_sample_t *s,bool *fresh)
{
    invalidate(s,platform_micros(),RANGE_STATUS_FAULT);*fresh=true;platform_tof_xshut(false);t->state=TOF_RETRY_WAIT;t->deadline_ms=now+SENSOR_RETRY_MS;
}

bool tof_step(tof_t *t, uint32_t now, range_sample_t *s, bool *fresh)
{
    *fresh=false;
    if (t->state==TOF_BOOT_WAIT && time_reached_u32(now,t->deadline_ms)) { platform_tof_xshut(true); t->state=TOF_INITIALIZING; t->deadline_ms=now+3U; t->init_index=0U; }
    if (t->state==TOF_INITIALIZING && time_reached_u32(now,t->deadline_ms)) {
        const uint16_t reg=(uint16_t)(TOF_CONFIG_FIRST+t->init_index);
        if(VL53L1_WrByte(VL53L1X_I2C_ADDRESS,reg,VL51L1X_DEFAULT_CONFIGURATION[t->init_index])!=0){enter_fault(t,now,s,fresh);return false;}
        if(reg==TOF_CONFIG_LAST){if(VL53L1X_StartRanging(VL53L1X_I2C_ADDRESS)!=0){enter_fault(t,now,s,fresh);return false;}t->state=TOF_CALIBRATING;t->timeout_ms=now+TOF_BOOT_TIMEOUT_MS;t->deadline_ms=now+1U;}
        else t->init_index++;
    }
    if(t->state==TOF_CALIBRATING&&time_reached_u32(now,t->deadline_ms)){
        uint8_t ready=0U;if(VL53L1X_CheckForDataReady(VL53L1X_I2C_ADDRESS,&ready)!=0){enter_fault(t,now,s,fresh);return false;}
        if(ready!=0U){if(VL53L1X_ClearInterrupt(VL53L1X_I2C_ADDRESS)!=0||VL53L1X_StopRanging(VL53L1X_I2C_ADDRESS)!=0||VL53L1_WrByte(VL53L1X_I2C_ADDRESS,VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,0x09U)!=0||VL53L1_WrByte(VL53L1X_I2C_ADDRESS,0x0BU,0U)!=0||!configure_continuous()){enter_fault(t,now,s,fresh);return false;}t->state=TOF_RUNNING;}
        else if(time_reached_u32(now,t->timeout_ms)){enter_fault(t,now,s,fresh);return false;}else t->deadline_ms=now+1U;
    }
    if (t->state==TOF_RUNNING && t->irq_pending) {
        VL53L1X_Result_t r; t->irq_pending=false;
        if (VL53L1X_GetResult(VL53L1X_I2C_ADDRESS,&r)!=0 || VL53L1X_ClearInterrupt(VL53L1X_I2C_ADDRESS)!=0) {
            invalidate(s,platform_micros(),RANGE_STATUS_FAULT); *fresh=true; platform_tof_xshut(false); t->state=TOF_RETRY_WAIT; t->deadline_ms=now+SENSOR_RETRY_MS; return false;
        }
        s->timestamp_us=platform_micros(); s->distance_mm=(int32_t)r.Distance+RANGE_OFFSET_MM; s->quality=r.Status==0U?100U:0U;
        if (r.Status==0U && s->distance_mm<RANGE_MIN_MM) s->status=RANGE_STATUS_TOO_CLOSE;
        else if (r.Status==0U && s->distance_mm>RANGE_MAX_MM) s->status=RANGE_STATUS_TOO_FAR;
        else if (r.Status==0U) s->status=RANGE_STATUS_VALID;
        else s->status=RANGE_STATUS_INVALID;
        *fresh=true;
    }
    if (t->state==TOF_RETRY_WAIT && time_reached_u32(now,t->deadline_ms)) { platform_tof_xshut(true); t->state=TOF_INITIALIZING; t->deadline_ms=now+3U; t->init_index=0U; }
    return t->state==TOF_RUNNING;
}
