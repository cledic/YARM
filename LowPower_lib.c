#include <power.h>
#include <port.h>

#include "conf_clocks.h"
#include "LowPower_lib.h"

static void main_clock_select_osc16m(void);
static void main_clock_select_dfll(void);
static void main_clock_select(const enum system_clock_source clock_source);
static void test_active_mode(const enum system_performance_level performance_level);


void ChkSleepOn( void)
{
	struct port_config pin_conf;
	
	/* PA08 as ChkSleep */
	port_pin_set_output_level( PIN_PA08, true);
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA08, &pin_conf);
}

void ChkSleepOff( void)
{
	struct port_config pin_conf;
	
	/* PA08 as ChkSleep */
//	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
//	port_pin_set_config(PIN_PA08, &pin_conf);
	port_pin_set_output_level( PIN_PA08, false);
}

/**
 * \brief Setect OSC16M as main clock source.
 */
static void main_clock_select_osc16m(void)
{
	struct system_gclk_gen_config gclk_conf;
	struct system_clock_source_osc16m_config osc16m_conf;

	/* Switch to new frequency selection and enable OSC16M */
	system_clock_source_osc16m_get_config_defaults(&osc16m_conf);
	osc16m_conf.fsel = CONF_CLOCK_OSC16M_FREQ_SEL;
	osc16m_conf.on_demand = 0;
	osc16m_conf.run_in_standby = CONF_CLOCK_OSC16M_RUN_IN_STANDBY;
	system_clock_source_osc16m_set_config(&osc16m_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_OSC16M);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_OSC16M));

	/* Select OSC16M as mainclock */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC16M;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);
	if (CONF_CLOCK_OSC16M_ON_DEMAND) {
		OSCCTRL->OSC16MCTRL.reg |= OSCCTRL_OSC16MCTRL_ONDEMAND;
	}

}

/**
 * \brief Setect DFLL as main clock source.
 */
static void main_clock_select_dfll(void)
{
	struct system_gclk_gen_config gclk_conf;

	/* Select OSCULP32K as new clock source for mainclock temporarily */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_XOSC32K;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);

	/* Select XOSC32K for GCLK1. */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_XOSC32K;
	system_gclk_gen_set_config(GCLK_GENERATOR_1, &gclk_conf);
	system_gclk_gen_enable(GCLK_GENERATOR_1);

	struct system_gclk_chan_config dfll_gclk_chan_conf;

	system_gclk_chan_get_config_defaults(&dfll_gclk_chan_conf);
	dfll_gclk_chan_conf.source_generator = GCLK_GENERATOR_1;
	system_gclk_chan_set_config(OSCCTRL_GCLK_ID_DFLL48, &dfll_gclk_chan_conf);
	system_gclk_chan_enable(OSCCTRL_GCLK_ID_DFLL48);

	struct system_clock_source_dfll_config dfll_conf;
	system_clock_source_dfll_get_config_defaults(&dfll_conf);

	dfll_conf.loop_mode      = SYSTEM_CLOCK_DFLL_LOOP_MODE_CLOSED;
	dfll_conf.on_demand      = false;
	dfll_conf.run_in_stanby  = CONF_CLOCK_DFLL_RUN_IN_STANDBY;
	dfll_conf.multiply_factor = CONF_CLOCK_DFLL_MULTIPLY_FACTOR;
	system_clock_source_dfll_set_config(&dfll_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DFLL);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_DFLL));
	if (CONF_CLOCK_DFLL_ON_DEMAND) {
		OSCCTRL->DFLLCTRL.bit.ONDEMAND = 1;
	}

	/* Select DFLL for mainclock. */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);

}


/**
 * \brief Main clock source selection between DFLL and OSC16M.
 */
static void main_clock_select(const enum system_clock_source clock_source)
{
	if (clock_source == SYSTEM_CLOCK_SOURCE_DFLL) {
		main_clock_select_dfll();
		system_clock_source_disable(SYSTEM_CLOCK_SOURCE_OSC16M);
	} else if (clock_source == SYSTEM_CLOCK_SOURCE_OSC16M) {
		main_clock_select_osc16m();
		system_clock_source_disable(SYSTEM_CLOCK_SOURCE_DFLL);
		system_gclk_chan_disable(OSCCTRL_GCLK_ID_DFLL48);
		system_gclk_gen_disable(GCLK_GENERATOR_1);
	} else {
		return ;
	}
}

/**
 * \brief Active mode test case.
 */
static void test_active_mode(const enum system_performance_level performance_level)
{

	enum system_performance_level curr_pl = system_get_performance_level();

	if (curr_pl == performance_level) {
		return ;
	}

	if (curr_pl < performance_level) {

		/* Scaling up the performance level first and then increase clock frequency */
		system_switch_performance_level(performance_level);
		main_clock_select(SYSTEM_CLOCK_SOURCE_DFLL);

	} else {
		/* Scaling down clock frequency and then Scaling down the performance level */
		main_clock_select(SYSTEM_CLOCK_SOURCE_OSC16M);
		system_switch_performance_level(performance_level);
	}

}

/**
 * \brief Idle mode test case.
 */
void EnterIdleMode(void)
{
	ChkSleepOff();
	
	// printf("Warning:System will enter IDLE mode,please wait until LED0 becomes OFF \r\n");
	test_active_mode(SYSTEM_PERFORMANCE_LEVEL_0);
	system_set_sleepmode(SYSTEM_SLEEPMODE_IDLE);
	system_sleep();

	ChkSleepOn();
	
}


/**
 * \brief STANDBY mode test case:Dynamic Power SleepWalking.
 *
 * era test_standby_mode_dynamic_power_sleepwalking
 */
void EnterSleepMode(void)
{

	// printf("System will enter STANDBY mode:Dynamic Power SleepWalking\r\n");

	/* When entering standby mode, the FDPLL is still running even if not
	 *	requested by any module causing extra consumption. Errata reference:12244
	 */
	test_active_mode(SYSTEM_PERFORMANCE_LEVEL_0);

	struct system_standby_config config;
	system_standby_get_config_defaults(&config);
	config.enable_dpgpd0 = true;
	config.enable_dpgpd1 = true;
	config.power_domain = SYSTEM_POWER_DOMAIN_DEFAULT;

	/* Errata 13599:
	 * In Standby mode, when Power Domain 1 is power gated,
	 * devices can show higher consumption than expected.
	*/
	config.power_domain = SYSTEM_POWER_DOMAIN_PD01;

	system_standby_set_config(&config);

	/* Errata 13901:
	 * In standby mode, when running modules from GCLK clock with 32Khz source,
	 * the main voltage regulator is used instead of the low power regulator,
	 * causing higher power consumption.
	*/
	if (system_get_performance_level() == SYSTEM_PERFORMANCE_LEVEL_0) {
		uint32_t *const tmp = (void *)(0x4000141C);
		*tmp |= (1 << 8);
	}

	system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
	system_sleep();

	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_XOSC32K);

	/* Errata 13901:
	 * In standby mode, when running modules from GCLK clock with 32Khz source,
	 * the main voltage regulator is used instead of the low power regulator,
	 * causing higher power consumption.
	*/
	if (system_get_performance_level() == SYSTEM_PERFORMANCE_LEVEL_0) {
		uint32_t *const tmp = (void *)(0x4000141C);
		*tmp &= ~(1 << 8);
	}

}

/**
 * \brief STANDBY mode test case: static power sleepwalking.
 *
 * era test_standby_mode_static_power_sleepwalking
*/
void EnterSleepMode2(void)
{

	ChkSleepOff();
	
	//spi_disable(&spi_master_instance);

	/* When entering standby mode, the FDPLL is still running even if not
	 *	requested by any module causing extra consumption. Errata reference:12244
	 */
	test_active_mode(SYSTEM_PERFORMANCE_LEVEL_0);

	system_clock_source_disable(SYSTEM_CLOCK_SOURCE_XOSC32K);

	/*
		When VDDCORE is supplied by the BUCK converter in performance
		level 0, the chip cannot wake-up from standby mode because the
		VCORERDY status is stuck at 0. Errata reference: 13551
	*/
	SUPC->VREG.bit.SEL = SUPC_VREG_SEL_LDO_Val;

	struct system_standby_config config;
	system_standby_get_config_defaults(&config);
	config.enable_dpgpd0       = false;
	config.enable_dpgpd1       = false;
	config.power_domain        = SYSTEM_POWER_DOMAIN_DEFAULT;
#if (SAML21XXXB)
	config.vregs_mode          = SYSTEM_SYSTEM_VREG_SWITCH_AUTO;
#else
	config.disable_avregsd     = false;
#endif
	config.linked_power_domain = SYSTEM_LINKED_POWER_DOMAIN_DEFAULT;
	config.hmcramchs_back_bias = SYSTEM_RAM_BACK_BIAS_STANDBY_OFF;
	config.hmcramclp_back_bias = SYSTEM_RAM_BACK_BIAS_STANDBY_OFF;

	system_standby_set_config(&config);

	/* Errata 13901:
	 * In standby mode, when running modules from GCLK clock with 32Khz source,
	 * the main voltage regulator is used instead of the low power regulator,
	 * causing higher power consumption.
	*/
	if (system_get_performance_level() == SYSTEM_PERFORMANCE_LEVEL_0) {
		uint32_t *const tmp = (void *)(0x4000141C);
		*tmp |= (1 << 8);
	}

	system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
	system_sleep();

	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_XOSC32K);

	//spi_enable(&spi_master_instance);
	
	/* Errata 13901:
	 * In standby mode, when running modules from GCLK clock with 32Khz source,
	 * the main voltage regulator is used instead of the low power regulator,
	 * causing higher power consumption.
	*/
	if (system_get_performance_level() == SYSTEM_PERFORMANCE_LEVEL_0) {
		uint32_t *const tmp = (void *)(0x4000141C);
		*tmp &= ~(1 << 8);
	}

	SUPC->VREG.bit.SEL = SUPC_VREG_SEL_BUCK_Val;
	
	ChkSleepOn();
}
