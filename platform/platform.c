#include "platform.h"
#include "config.h"
#include "rtos_app.h"
#include "flowcan/ring.h"
#include "FreeRTOS.h"
#include "task.h"
#include "at32f415.h"
#include "at32f415_can.h"
#include "at32f415_crm.h"
#include "at32f415_dma.h"
#include "at32f415_exint.h"
#include "at32f415_gpio.h"
#include "at32f415_i2c.h"
#include "at32f415_misc.h"
#include "at32f415_spi.h"
#include "at32f415_tmr.h"
#include "at32f415_usart.h"
#include "at32f415_wdt.h"
#include <string.h>

#define UART_RING_SIZE 256U
#define CAN_RING_SIZE CAN_RX_QUEUE_SIZE
#define WS_BITS 48U
#define CAN_TIME_QUANTA 18UL
#define CAN_BAUD_DIVIDER (APB1_HZ / (CAN_BITRATE * CAN_TIME_QUANTA))
#define WATCHDOG_RELOAD_VALUE \
    ((((WATCHDOG_TIMEOUT_MS * WATCHDOG_CLOCK_HZ) + (1000UL * WATCHDOG_DIVIDER) - 1UL) / \
      (1000UL * WATCHDOG_DIVIDER)) - 1UL)
#if WATCHDOG_RELOAD_VALUE > 4095UL
#error "WATCHDOG_TIMEOUT_MS exceeds the 12-bit watchdog reload range"
#endif
#if WATCHDOG_DIVIDER == 256UL
#define WATCHDOG_DIVIDER_ENUM WDT_CLK_DIV_256
#else
#error "Unsupported WATCHDOG_DIVIDER"
#endif
#if PMW3901_SPI_DIV == 64U
#define PMW3901_SPI_DIVIDER_ENUM SPI_MCLK_DIV_64
#else
#error "Unsupported PMW3901_SPI_DIV"
#endif
static volatile uint8_t uart_rx[UART_RING_SIZE],uart_tx[UART_RING_SIZE];
static volatile uint16_t urx_head,urx_tail,utx_head,utx_tail;
typedef struct{CanardCANFrame frame;uint64_t timestamp;}can_slot_t;
static volatile can_slot_t can_rx[CAN_RING_SIZE];
static volatile uint8_t can_head,can_tail;
static volatile bool can_overflow,can_activity;
#if FEATURE_WS2812
static uint16_t ws_dma[WS_BITS];
#endif
static volatile bool ws_busy;

static void gpio_output(gpio_type *port,uint16_t pins,bool high)
{
    gpio_init_type g;gpio_default_para_init(&g);g.gpio_pins=pins;g.gpio_mode=GPIO_MODE_OUTPUT;g.gpio_out_type=GPIO_OUTPUT_PUSH_PULL;g.gpio_pull=GPIO_PULL_NONE;g.gpio_drive_strength=GPIO_DRIVE_STRENGTH_STRONGER;gpio_init(port,&g);gpio_bits_write(port,pins,high?TRUE:FALSE);
}
static void gpio_mux(gpio_type *port,uint16_t pins,gpio_pull_type pull)
{
    gpio_init_type g;gpio_default_para_init(&g);g.gpio_pins=pins;g.gpio_mode=GPIO_MODE_MUX;g.gpio_out_type=GPIO_OUTPUT_PUSH_PULL;g.gpio_pull=pull;g.gpio_drive_strength=GPIO_DRIVE_STRENGTH_STRONGER;gpio_init(port,&g);
}
static void gpio_mux_open_drain(gpio_type *port,uint16_t pins)
{
    gpio_init_type g;gpio_default_para_init(&g);g.gpio_pins=pins;g.gpio_mode=GPIO_MODE_MUX;g.gpio_out_type=GPIO_OUTPUT_OPEN_DRAIN;g.gpio_pull=GPIO_PULL_UP;g.gpio_drive_strength=GPIO_DRIVE_STRENGTH_STRONGER;gpio_init(port,&g);
}

static void init_gpio(void)
{
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK,TRUE);crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK,TRUE);crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK,TRUE);
    gpio_output(GPIOA,GPIO_PINS_0|GPIO_PINS_1|GPIO_PINS_2|GPIO_PINS_3, true);
    gpio_output(GPIOA,GPIO_PINS_4,true);gpio_output(GPIOB,GPIO_PINS_0|GPIO_PINS_5,false);
    gpio_mux(GPIOA,GPIO_PINS_5|GPIO_PINS_6|GPIO_PINS_7,GPIO_PULL_NONE);
    gpio_init_type in;gpio_default_para_init(&in);in.gpio_pins=GPIO_PINS_1|GPIO_PINS_8;in.gpio_mode=GPIO_MODE_INPUT;in.gpio_pull=GPIO_PULL_UP;gpio_init(GPIOB,&in);
    gpio_mux_open_drain(GPIOB,GPIO_PINS_6|GPIO_PINS_7);gpio_mux(GPIOA,GPIO_PINS_8|GPIO_PINS_9|GPIO_PINS_12,GPIO_PULL_NONE);
    gpio_default_para_init(&in);in.gpio_pins=GPIO_PINS_10|GPIO_PINS_11;in.gpio_mode=GPIO_MODE_INPUT;in.gpio_pull=GPIO_PULL_UP;gpio_init(GPIOA,&in);
}

static void init_exti(void)
{
    gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOB,GPIO_PINS_SOURCE1);gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOB,GPIO_PINS_SOURCE8);
    exint_init_type e;exint_default_para_init(&e);e.line_mode=EXINT_LINE_INTERRUPUT;e.line_polarity=EXINT_TRIGGER_FALLING_EDGE;e.line_enable=TRUE;e.line_select=EXINT_LINE_1;exint_init(&e);e.line_select=EXINT_LINE_8;exint_init(&e);
    nvic_irq_enable(EXINT1_IRQn,6U,0U);nvic_irq_enable(EXINT9_5_IRQn,6U,0U);
}
static void init_spi(void)
{
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK,TRUE);spi_init_type s;spi_default_para_init(&s);s.master_slave_mode=SPI_MODE_MASTER;s.transmission_mode=SPI_TRANSMIT_FULL_DUPLEX;s.first_bit_transmission=SPI_FIRST_BIT_MSB;s.mclk_freq_division=PMW3901_SPI_DIVIDER_ENUM;s.frame_bit_num=SPI_FRAME_8BIT;s.cs_mode_selection=SPI_CS_SOFTWARE_MODE;s.clock_polarity=SPI_CLOCK_POLARITY_HIGH;s.clock_phase=SPI_CLOCK_PHASE_2EDGE;spi_init(SPI1,&s);spi_software_cs_internal_level_set(SPI1,SPI_SWCS_INTERNAL_LEVEL_HIGHT);spi_enable(SPI1,TRUE);
}
static void init_i2c(void)
{
    crm_periph_clock_enable(CRM_I2C1_PERIPH_CLOCK,TRUE);i2c_init(I2C1,I2C_FSMODE_DUTY_2_1,VL53L1X_I2C_HZ);i2c_own_address1_set(I2C1,I2C_ADDRESS_MODE_7BIT,0U);i2c_ack_enable(I2C1,TRUE);i2c_enable(I2C1,TRUE);
}
#if FEATURE_MSP
static void init_uart(void)
{
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK,TRUE);usart_init(USART1,MSP_BAUDRATE,USART_DATA_8BITS,USART_STOP_1_BIT);usart_parity_selection_config(USART1,USART_PARITY_NONE);usart_transmitter_enable(USART1,TRUE);usart_receiver_enable(USART1,TRUE);usart_interrupt_enable(USART1,USART_RDBF_INT,TRUE);usart_enable(USART1,TRUE);nvic_irq_enable(USART1_IRQn,7U,0U);
}
#endif
static void init_can(void)
{
    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK,TRUE);gpio_pin_remap_config(CAN1_GMUX_0000,TRUE);
    can_base_type b;can_default_para_init(&b);b.mode_selection=CAN_MODE_COMMUNICATE;b.aebo_enable=TRUE;b.aed_enable=TRUE;b.mdrsel_selection=CAN_DISCARDING_FIRST_RECEIVED;b.mmssr_selection=CAN_SENDING_BY_REQUEST;(void)can_base_init(CAN1,&b);
    can_baudrate_type baud={.baudrate_div=(uint16_t)CAN_BAUD_DIVIDER,.rsaw_size=CAN_RSAW_3TQ,.bts1_size=CAN_BTS1_13TQ,.bts2_size=CAN_BTS2_4TQ};(void)can_baudrate_set(CAN1,&baud);
    can_filter_init_type f={.filter_activate_enable=TRUE,.filter_fifo=CAN_FILTER_FIFO0,.filter_number=0U,.filter_bit=CAN_FILTER_32BIT,.filter_id_high=0U,.filter_id_low=0U,.filter_mask_high=0U,.filter_mask_low=0U};can_filter_init(CAN1,&f);
    can_interrupt_enable(CAN1,CAN_RF0MIEN_INT,TRUE);can_interrupt_enable(CAN1,CAN_ETRIEN_INT,TRUE);can_interrupt_enable(CAN1,CAN_EOIEN_INT,TRUE);nvic_irq_enable(CAN1_RX0_IRQn,5U,0U);nvic_irq_enable(CAN1_SE_IRQn,5U,0U);
}
static void init_timers(void)
{
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK,TRUE);tmr_base_init(TMR2,0xFFFFFFFFUL,(SYSCLK_HZ/TMR2_HZ)-1U);tmr_32_bit_function_enable(TMR2,TRUE);tmr_counter_enable(TMR2,TRUE);
#if FEATURE_WS2812
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK,TRUE);tmr_base_init(TMR1,179U,0U);tmr_output_config_type o;tmr_output_default_para_init(&o);o.oc_mode=TMR_OUTPUT_CONTROL_PWM_MODE_A;o.oc_output_state=TRUE;o.oc_polarity=TMR_OUTPUT_ACTIVE_HIGH;tmr_output_channel_config(TMR1,TMR_SELECT_CHANNEL_1,&o);tmr_channel_enable(TMR1,TMR_SELECT_CHANNEL_1,TRUE);tmr_output_enable(TMR1,TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK,TRUE);dma_flexible_config(DMA1,FLEX_CHANNEL2,DMA_FLEXIBLE_TMR1_OVERFLOW);dma_init_type d;dma_default_para_init(&d);d.buffer_size=WS_BITS;d.direction=DMA_DIR_MEMORY_TO_PERIPHERAL;d.memory_base_addr=(uint32_t)ws_dma;d.memory_data_width=DMA_MEMORY_DATA_WIDTH_HALFWORD;d.memory_inc_enable=TRUE;d.peripheral_base_addr=(uint32_t)&TMR1->c1dt;d.peripheral_data_width=DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;d.peripheral_inc_enable=FALSE;d.priority=DMA_PRIORITY_MEDIUM;d.loop_mode_enable=FALSE;dma_init(DMA1_CHANNEL2,&d);dma_interrupt_enable(DMA1_CHANNEL2,DMA_FDT_INT,TRUE);tmr_dma_request_enable(TMR1,TMR_OVERFLOW_DMA_REQUEST,TRUE);nvic_irq_enable(DMA1_Channel2_IRQn,8U,0U);
#endif
}

void platform_init(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);init_gpio();init_exti();init_spi();init_i2c();
#if FEATURE_MSP
    init_uart();
#endif
#if FEATURE_DRONECAN
    init_can();
#endif
    init_timers();
    wdt_register_write_enable(TRUE);wdt_divider_set(WATCHDOG_DIVIDER_ENUM);wdt_reload_value_set((uint16_t)WATCHDOG_RELOAD_VALUE);wdt_counter_reload();wdt_enable();
}
uint32_t platform_millis(void){return (uint32_t)xTaskGetTickCount();}uint32_t platform_micros(void){return tmr_counter_value_get(TMR2);}void platform_delay_us(uint32_t us){uint32_t s=platform_micros();while((uint32_t)(platform_micros()-s)<us){}}
void platform_watchdog_feed(void){wdt_counter_reload();}
bool platform_spi_transfer(uint8_t v,uint8_t *received){uint32_t start=platform_micros();if(received==NULL)return false;while(spi_i2s_flag_get(SPI1,SPI_I2S_TDBE_FLAG)==RESET){if((uint32_t)(platform_micros()-start)>=PMW3901_SPI_TIMEOUT_US){spi_i2s_reset(SPI1);init_spi();return false;}}spi_i2s_data_transmit(SPI1,v);start=platform_micros();while(spi_i2s_flag_get(SPI1,SPI_I2S_RDBF_FLAG)==RESET){if((uint32_t)(platform_micros()-start)>=PMW3901_SPI_TIMEOUT_US){spi_i2s_reset(SPI1);init_spi();return false;}}*received=(uint8_t)spi_i2s_data_receive(SPI1);return true;}
void platform_pmw_cs(bool a){gpio_bits_write(GPIOA,GPIO_PINS_4,a?FALSE:TRUE);}void platform_pmw_reset(bool a){gpio_bits_write(GPIOB,GPIO_PINS_0,a?FALSE:TRUE);}
void platform_tof_xshut(bool e){gpio_bits_write(GPIOB,GPIO_PINS_5,e?TRUE:FALSE);}

static bool i2c_error_present(void){return i2c_flag_get(I2C1,I2C_BUSERR_FLAG)!=RESET||i2c_flag_get(I2C1,I2C_ARLOST_FLAG)!=RESET||i2c_flag_get(I2C1,I2C_ACKFAIL_FLAG)!=RESET||i2c_flag_get(I2C1,I2C_OUF_FLAG)!=RESET||i2c_flag_get(I2C1,I2C_TMOUT_FLAG)!=RESET;}
static bool wait_i2c(uint32_t flag,uint32_t timeout){uint32_t s=platform_micros();while(i2c_flag_get(I2C1,flag)==RESET){if(i2c_error_present()||(uint32_t)(platform_micros()-s)>=timeout)return false;}return true;}
static bool i2c_fail(void){i2c_stop_generate(I2C1);i2c_reset(I2C1);init_i2c();return false;}
bool platform_i2c_write(uint8_t addr,const uint8_t *data,size_t len,uint32_t timeout)
{
    if(data==NULL&&len!=0U)return false;
    uint32_t s=platform_micros();while(i2c_flag_get(I2C1,I2C_BUSYF_FLAG)!=RESET){if((uint32_t)(platform_micros()-s)>=timeout)return i2c_fail();}i2c_master_receive_ack_set(I2C1,I2C_MASTER_ACK_CURRENT);i2c_start_generate(I2C1);if(!wait_i2c(I2C_STARTF_FLAG,timeout))return i2c_fail();i2c_7bit_address_send(I2C1,addr,I2C_DIRECTION_TRANSMIT);if(!wait_i2c(I2C_ADDR7F_FLAG,timeout))return i2c_fail();i2c_flag_clear(I2C1,I2C_ADDR7F_FLAG);
    for(size_t i=0;i<len;i++){if(!wait_i2c(I2C_TDBE_FLAG,timeout))return i2c_fail();i2c_data_send(I2C1,data[i]);}if(!wait_i2c(I2C_TDC_FLAG,timeout))return i2c_fail();i2c_stop_generate(I2C1);return true;
}
bool platform_i2c_read(uint8_t addr,uint8_t *data,size_t len,uint32_t timeout)
{
    if(len==0U)return true;
    if(data==NULL)return false;
    uint32_t s=platform_micros();while(i2c_flag_get(I2C1,I2C_BUSYF_FLAG)!=RESET){if((uint32_t)(platform_micros()-s)>=timeout)return i2c_fail();}i2c_master_receive_ack_set(I2C1,I2C_MASTER_ACK_CURRENT);i2c_ack_enable(I2C1,TRUE);i2c_start_generate(I2C1);if(!wait_i2c(I2C_STARTF_FLAG,timeout))return i2c_fail();i2c_7bit_address_send(I2C1,addr,I2C_DIRECTION_RECEIVE);if(!wait_i2c(I2C_ADDR7F_FLAG,timeout))return i2c_fail();
    if(len==1U){i2c_ack_enable(I2C1,FALSE);i2c_flag_clear(I2C1,I2C_ADDR7F_FLAG);i2c_stop_generate(I2C1);if(!wait_i2c(I2C_RDBF_FLAG,timeout))return i2c_fail();data[0]=i2c_data_receive(I2C1);}
    else if(len==2U){i2c_master_receive_ack_set(I2C1,I2C_MASTER_ACK_NEXT);i2c_ack_enable(I2C1,FALSE);i2c_flag_clear(I2C1,I2C_ADDR7F_FLAG);if(!wait_i2c(I2C_TDC_FLAG,timeout))return i2c_fail();i2c_stop_generate(I2C1);data[0]=i2c_data_receive(I2C1);data[1]=i2c_data_receive(I2C1);}
    else{i2c_flag_clear(I2C1,I2C_ADDR7F_FLAG);size_t remaining=len;size_t index=0U;while(remaining>3U){if(!wait_i2c(I2C_RDBF_FLAG,timeout))return i2c_fail();data[index++]=i2c_data_receive(I2C1);remaining--;}if(!wait_i2c(I2C_TDC_FLAG,timeout))return i2c_fail();i2c_ack_enable(I2C1,FALSE);data[index++]=i2c_data_receive(I2C1);if(!wait_i2c(I2C_TDC_FLAG,timeout))return i2c_fail();i2c_stop_generate(I2C1);data[index++]=i2c_data_receive(I2C1);data[index]=i2c_data_receive(I2C1);}
    i2c_master_receive_ack_set(I2C1,I2C_MASTER_ACK_CURRENT);i2c_ack_enable(I2C1,TRUE);return true;
}
bool platform_uart_tx(const uint8_t *data,size_t len)
{
    const uint16_t head=utx_head,tail=utx_tail;
    if(data==NULL&&len!=0U)return false;
    if(!ring_can_write(UART_RING_SIZE,head,tail,len))return false;
    uint16_t next=head;
    for(size_t i=0;i<len;i++){uart_tx[next]=data[i];next=(uint16_t)((next+1U)%UART_RING_SIZE);}
    utx_head=next;usart_interrupt_enable(USART1,USART_TDBE_INT,TRUE);return true;
}
bool platform_uart_rx_pop(uint8_t *b){if(urx_tail==urx_head)return false;*b=uart_rx[urx_tail];urx_tail=(uint16_t)((urx_tail+1U)%UART_RING_SIZE);return true;}
bool platform_can_tx(const CanardCANFrame *f){if(f==NULL||f->data_len>CANARD_CAN_FRAME_MAX_DATA_LEN||(f->id&CANARD_CAN_FRAME_EFF)==0U)return false;can_tx_message_type t={0};t.id_type=CAN_ID_EXTENDED;t.extended_id=f->id&CANARD_CAN_EXT_ID_MASK;t.frame_type=CAN_TFT_DATA;t.dlc=f->data_len;memcpy(t.data,f->data,f->data_len);return can_message_transmit(CAN1,&t)!=CAN_TX_STATUS_NO_EMPTY;}
bool platform_can_rx_pop(CanardCANFrame *f,uint64_t *ts){if(can_tail==can_head)return false;*f=can_rx[can_tail].frame;*ts=can_rx[can_tail].timestamp;can_tail=(uint8_t)((can_tail+1U)%CAN_RING_SIZE);return true;}
bool platform_can_bus_off(void){return can_flag_get(CAN1,CAN_BOF_FLAG)==SET;}void platform_can_reinit(void){nvic_irq_disable(CAN1_RX0_IRQn);nvic_irq_disable(CAN1_SE_IRQn);can_reset(CAN1);can_head=0U;can_tail=0U;can_activity=false;init_can();}
static bool take_flag(volatile bool *flag){uint32_t primask=__get_PRIMASK();__disable_irq();bool value=*flag;*flag=false;if(primask==0U)__enable_irq();return value;}
bool platform_take_can_rx_overflow(void){return take_flag(&can_overflow);}
bool platform_take_can_rx_activity(void){return take_flag(&can_activity);}
void platform_activity_led(activity_led_t led,bool on){gpio_bits_write(GPIOA,(uint16_t)(GPIO_PINS_0<<(uint8_t)led),on?FALSE:TRUE);}
bool platform_ws2812_start(uint8_t r,uint8_t g,uint8_t b)
{
#if FEATURE_WS2812
    if(ws_busy)return false;
    uint32_t grb=((uint32_t)g<<16U)|((uint32_t)r<<8U)|b;for(unsigned i=0;i<24U;i++)ws_dma[i]=(grb&(1UL<<(23U-i)))?120U:60U;for(unsigned i=24U;i<WS_BITS;i++)ws_dma[i]=0U;DMA1_CHANNEL2->ctrl_bit.chen=FALSE;DMA1_CHANNEL2->dtcnt=WS_BITS;tmr_counter_value_set(TMR1,0U);ws_busy=true;DMA1_CHANNEL2->ctrl_bit.chen=TRUE;tmr_counter_enable(TMR1,TRUE);return true;
#else
    (void)r;(void)g;(void)b;return false;
#endif
}
bool platform_ws2812_busy(void){return ws_busy;}

void EXINT1_IRQHandler(void){if(exint_flag_get(EXINT_LINE_1)!=RESET){exint_flag_clear(EXINT_LINE_1);rtos_notify_flow_from_isr();}}
void EXINT9_5_IRQHandler(void){if(exint_flag_get(EXINT_LINE_8)!=RESET){exint_flag_clear(EXINT_LINE_8);rtos_notify_range_from_isr();}}
void USART1_IRQHandler(void)
{
    if(usart_flag_get(USART1,USART_RDBF_FLAG)!=RESET){uint16_t n=(uint16_t)((urx_head+1U)%UART_RING_SIZE);uint8_t b=(uint8_t)usart_data_receive(USART1);if(n!=urx_tail){uart_rx[urx_head]=b;urx_head=n;}rtos_notify_communication_from_isr();}
    if(usart_flag_get(USART1,USART_TDBE_FLAG)!=RESET){if(utx_tail==utx_head)usart_interrupt_enable(USART1,USART_TDBE_INT,FALSE);else{usart_data_transmit(USART1,uart_tx[utx_tail]);utx_tail=(uint16_t)((utx_tail+1U)%UART_RING_SIZE);}}
}
void CAN1_RX0_IRQHandler(void)
{
    can_rx_message_type r;while(CAN1->rf0_bit.rf0mn!=0U){can_message_receive(CAN1,CAN_RX_FIFO0,&r);can_activity=true;if(r.id_type!=CAN_ID_EXTENDED||r.frame_type!=CAN_TFT_DATA||r.dlc>CANARD_CAN_FRAME_MAX_DATA_LEN){can_overflow=true;continue;}uint8_t n=(uint8_t)((can_head+1U)%CAN_RING_SIZE);if(n==can_tail){can_overflow=true;continue;}can_slot_t *s=(can_slot_t *)&can_rx[can_head];s->frame.id=r.extended_id|CANARD_CAN_FRAME_EFF;s->frame.data_len=r.dlc;memcpy(s->frame.data,r.data,r.dlc);s->timestamp=platform_micros();can_head=n;}rtos_notify_communication_from_isr();
}
void CAN1_SE_IRQHandler(void){can_flag_clear(CAN1,CAN_EOIF_FLAG);rtos_notify_communication_from_isr();}
void DMA1_Channel2_IRQHandler(void){if(dma_flag_get(DMA1_FDT2_FLAG)!=RESET){dma_flag_clear(DMA1_FDT2_FLAG);DMA1_CHANNEL2->ctrl_bit.chen=FALSE;tmr_counter_enable(TMR1,FALSE);ws_busy=false;}}
