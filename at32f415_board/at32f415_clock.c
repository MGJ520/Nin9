#include "at32f415_clock.h"

void system_clock_config(void)
{
  crm_reset();

  /* 140MHz requires 4 wait cycles */
  flash_psr_set(FLASH_WAIT_CYCLE_4);

  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  while(crm_hext_stable_wait() == ERROR);

  /* 20MHz / 2 = 10MHz, *14 = 140MHz */
  crm_pll_config(CRM_PLL_SOURCE_HEXT_DIV, CRM_PLL_MULT_14);

  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb2_div_set(CRM_APB2_DIV_2);   /* APB2 = 70MHz */
  crm_apb1_div_set(CRM_APB1_DIV_2);   /* APB1 = 70MHz */

  crm_auto_step_mode_enable(TRUE);
  crm_sysclk_switch(CRM_SCLK_PLL);
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL);
  crm_auto_step_mode_enable(FALSE);

  system_core_clock_update();
}

