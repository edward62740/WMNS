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

//log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//drivers
#include "clocks.h"
#include "battery.h"
#include "twi.h"
#include "mesh.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

void app_mesh_handler(message_t *msg) {
	NRF_LOG_INFO("app_mesh_handler()");
}

void app_rtc_handler() {
	//*(volatile uint32_t *)0x40000EE4 = (*(volatile uint32_t *)0x10000258 & 0x0000004F);
	clocks_restart();
	twi_restart();
	nrf_gpio_cfg_output (B_LED);
	nrf_gpio_pin_clear(B_LED);

	static uint32_t cycle_count = 0;
	static const uint32_t period_co2 = 1;
	static const uint32_t offset_co2 = 0;
	static const uint32_t period_bat = 2;
	static const uint32_t offset_bat = 0;
	static const uint32_t period_alive = 1;
	static const uint32_t offset_alive = 0;

	if (((cycle_count + offset_co2) % period_co2) == 0) {

		nrf_gpio_cfg_input(PG_IND, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(CHG_IND, NRF_GPIO_PIN_NOPULL);
		uint8_t stat = 0;
		if (nrf_gpio_pin_read (PG_IND)) {
			stat++;
		}
		if (nrf_gpio_pin_read (CHG_IND)) {
			stat++;
		}
		uint16_t co2;
		int32_t temp;
		int32_t hum;
		scd4x_read_measurement(&co2, &temp, &hum);
		mesh_tx_co2(temp, (uint32_t) hum, 0, (uint32_t) co2, stat);
		mesh_wait_tx();
		scd4x_measure_single_shot(); // try default alive after single shot
	}
	nrf_delay_ms(5); // check if needed
	/* Measure battery voltage and transmit message if bat period */
	if (((cycle_count + offset_bat) % period_bat) == 0) {
		uint16_t v_bat_mili = get_battery();
		mesh_tx_battery(v_bat_mili);
		mesh_wait_tx();

	}

	/* Transmit alive packet if alive period */
	else if (((cycle_count + offset_alive) % period_alive) == 0) {
		mesh_tx_alive();
		mesh_wait_tx();

	}

	/* Initialize GPIOTE and start measurement if env period */

	nrf_gpio_pin_clear(B_LED);
	nrf_gpio_cfg_default(B_LED);
	cycle_count++;
	twi_stop();
	clocks_stop();
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
	mesh_ttl_set(3);
	mesh_tx_reset();
	mesh_wait_tx();
	sensirion_i2c_hal_init(&m_twi);
	// Clean up potential SCD40 states
	scd4x_wake_up();
	scd4x_stop_periodic_measurement();
	scd4x_reinit();
	nrf_delay_ms(100);
	uint16_t status = 0;
	scd4x_perform_self_test(&status);

	while (status) {
		mesh_tx_battery(status);
		mesh_wait_tx();
		nrf_gpio_cfg_output (R_LED);
		nrf_gpio_pin_clear(R_LED);
		nrf_delay_ms(150);
		scd4x_perform_self_test(&status);
	}

	nrf_gpio_cfg_output (G_LED);
	nrf_delay_ms(300);
	nrf_gpio_pin_clear(G_LED);
	nrf_delay_ms(500);

	scd4x_measure_single_shot();
	rtc_config(app_rtc_handler);
	twi_stop();
	clocks_stop(); //release the hf clock

	nrf_gpio_cfg_default (R_LED);
	nrf_gpio_cfg_default (B_LED);
	nrf_gpio_cfg_default(G_LED);
	//NRF_TIMER0-> TASKS_SHUTDOWN = 1;
	while (true) {

		nrf_pwr_mgmt_run();

	}
}

