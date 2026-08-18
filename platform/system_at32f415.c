#include "at32f415.h"
#include "at32f415_crm.h"
#include "at32f415_flash.h"
#include "config.h"

uint32_t system_core_clock = HSE_HZ;

void SystemInit(void)
{
    crm_reset();
    flash_psr_set(FLASH_WAIT_CYCLE_4);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    while (crm_hext_stable_wait() != SUCCESS) { }
    crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_18);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) == RESET) { }
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_2);
    crm_apb2_div_set(CRM_APB2_DIV_2);
    crm_sysclk_switch(CRM_SCLK_PLL);
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) { }
    SCB->VTOR = 0x08000000UL;
    system_core_clock = SYSCLK_HZ;
}

void SystemCoreClockUpdate(void)
{
    system_core_clock = SYSCLK_HZ;
}
