#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_util.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_pwr_mgmt.h"
#include "math.h"
//log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//drivers
#include "ltr390.h"
#include "veml6030.h"
#include "clocks.h"
#include "battery.h"
#include "twi.h"
#include "mesh.h"

bool autorange_lv = 0;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

void app_mesh_handler(message_t *msg)
{
    NRF_LOG_INFO("app_mesh_handler()");
}

void app_rtc_handler()
{
    *(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
                                        & 0x0000004F);
    clocks_restart();
    twi_restart();


    /* Declare static counts */
    static uint32_t cycle_count = 0;
    static const uint32_t period_als = 1;
    static const uint32_t offset_als = 0;
    static const uint32_t period_bat = 60;
    static const uint32_t offset_bat = 1;
    static const uint32_t period_alive = 10;
    static const uint32_t offset_alive = 5;

    /* Measure battery voltage and transmit message if bat period */
    if (((cycle_count + offset_bat) % period_bat) == 0)
    {

        nrf_gpio_cfg_output(17);
        nrf_gpio_pin_clear(17);
        uint16_t v_bat_mili = get_battery();
        mesh_tx_battery(v_bat_mili);
#if(NRF_LOG_LEVEL <= NRF_LOG_LEVEL_INFO)
        float v_bat_mili_f = v_bat_mili;
        NRF_LOG_INFO("V Bat = "NRF_LOG_FLOAT_MARKER" Volts",NRF_LOG_FLOAT(v_bat_mili_f/1000));
#endif
		mesh_wait_tx();

		nrf_gpio_pin_set(17);
		nrf_gpio_cfg_default(17);
	}

	/* Transmit alive packet if alive period */
	else if (((cycle_count + offset_alive) % period_alive) == 0) {
		nrf_gpio_cfg_output(20);
		nrf_gpio_pin_clear(20);
		mesh_tx_alive();
		mesh_wait_tx();

		nrf_gpio_pin_set(20);
		nrf_gpio_cfg_default(20);
	}
	/* Initialize GPIOTE and start measurement if env period */
	else if (((cycle_count + offset_als) % period_als) == 0) {
		nrf_gpio_cfg_output(19);
		nrf_gpio_pin_clear(19);

        veml6030_power_on();
        //veml6030_int_en(1);
        ltr390_enable();



	}
	nrf_gpio_pin_set(19);
	nrf_gpio_cfg_default(19);
	cycle_count++;
	twi_stop();
	clocks_stop();
}

void GPIOTE_IRQHandler(void)
{
    *(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
                                        & 0x0000004F);
    clocks_restart();
    twi_restart();
    NVIC_DisableIRQ (GPIOTE_IRQn);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    if (NRF_GPIOTE->EVENTS_PORT)
    {
        NRF_GPIOTE->EVENTS_PORT = 0;
        uint16_t info  = veml6030_int_clear();

        nrf_gpio_cfg_output(19);
        nrf_gpio_pin_clear(19);
        nrf_gpio_cfg_output(17);
        nrf_gpio_pin_clear(17);

        double als, w;
        uint32_t uv;
        veml6030_read_res(&als, &w);

        uv = ltr390_readUVS();
        mesh_tx_als(als * 1000, 1000, w * 1000, uv);
        mesh_wait_tx();
        nrf_gpio_pin_set(19);
        nrf_gpio_cfg_default(19);
        nrf_gpio_pin_set(17);
        nrf_gpio_cfg_default(17);
        ltr390_disable();
        veml6030_shutdown();

    }

    nrf_gpio_cfg_sense_input(30, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt
    NVIC_EnableIRQ(GPIOTE_IRQn);
    twi_stop();
    clocks_stop(); //release the hf clock

}

int main(void) {
	*(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
			& 0x0000004F);

	uint32_t err_code;
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();

	err_code = nrf_pwr_mgmt_init();
	APP_ERROR_CHECK(err_code);
	for (int i = 0; i < 32; i++) {
		nrf_gpio_cfg_default(i);
	}
//NRF_POWER->DCDCEN = 1;
	clocks_start();
	battery_init();
	err_code = mesh_init(app_mesh_handler, NULL); //in case of rf rx, cmd handler not required
	APP_ERROR_CHECK(err_code);
	twi_init(&m_twi);
	rtc_config(app_rtc_handler);
	mesh_ttl_set(3);

    ltr390_init(&m_twi);
    nrf_delay_ms(5);
        ltr390_setMode(LTR390_MODE_UVS);
    ltr390_setGain(LTR390_GAIN_3);
    ltr390_setResolution(LTR390_RESOLUTION_18BIT);
    ltr390_enable();


    mesh_tx_battery(twi_scan());
	mesh_wait_tx();

	veml6030_init(&m_twi);
	nrf_delay_ms(5);
	veml6030_set_als_integration_time (REG_ALS_CONF_IT50);
	veml6030_set_als_gain (REG_ALS_CONF_GAIN_1_8);
	veml6030_set_psm (REG_POWER_SAVING_PSM_3);
    veml6030_int_en(1);

	twi_stop();

     NVIC_ClearPendingIRQ (GPIOTE_IRQn);
	 nrf_gpio_cfg_sense_input(30, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	 NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	 NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt
	 NVIC_EnableIRQ(GPIOTE_IRQn);


	clocks_stop(); //release the hf clock
//NRF_TIMER0-> TASKS_SHUTDOWN = 1;
	while (true) {
		nrf_pwr_mgmt_run();
	}
}

