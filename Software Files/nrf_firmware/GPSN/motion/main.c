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

#include "nrf_pwr_mgmt.h"

//for the log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_rtt.h"

// --------------------- inputs from sdk_config ---------------------
// ---> TWI0_ENABLED ---> TWI1_ENABLED

//drivers
#include "bma400.h"
//apps
#include "clocks.h"
#include "battery.h"
#include "twi.h"
#include "mesh.h"

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

volatile bool triggered = false;
volatile bool stat = false;
volatile uint32_t prev_time;
volatile uint32_t shock_count = 0;

void app_mesh_handler(message_t *msg) {
	*(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
			& 0x0000004F);
	NRF_LOG_INFO("app_mesh_handler()");
}

void app_rtc_handler() {
	*(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
			& 0x0000004F);
	clocks_restart();
	nrf_gpio_cfg_output (STAT_LED);
	nrf_gpio_pin_set(STAT_LED);
	/* Declare static counts */
	static uint32_t cycle_count = 0;
	static const uint32_t period_bat = 6;
	static const uint32_t offset_bat = 0;
	static const uint32_t period_alive = 1;
	static const uint32_t offset_alive = 0;

	/* Measure battery voltage and transmit message if bat period */


	if (((cycle_count + offset_bat) % period_bat) == 0) {
		uint16_t v_bat_mili = get_battery();
		mesh_tx_battery(v_bat_mili);
		mesh_wait_tx();
	}
	else if (((cycle_count + offset_alive) % period_alive) == 0) {
		mesh_tx_alive();
		mesh_wait_tx();
	}

	/* Transmit alive packet if alive period */

	cycle_count++;
	nrf_gpio_pin_clear(STAT_LED);
	clocks_stop();
}

void GPIOTE_IRQHandler(void) {
	*(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
			& 0x0000004F);
	clocks_restart();
	twi_restart();
	nrf_gpio_cfg_output (STAT_LED);
	nrf_gpio_pin_set(STAT_LED);
	NVIC_DisableIRQ (GPIOTE_IRQn);
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
	if (NRF_GPIOTE->EVENTS_PORT) {
		NRF_GPIOTE->EVENTS_PORT = 0;

		//NRF_LOG_INFO("interrupt triggered");
		if (!triggered) {
			nrf_gpio_cfg_sense_input(BMA_INT, NRF_GPIO_PIN_NOPULL,
					NRF_GPIO_PIN_SENSE_LOW);
			prev_time = bma400_get_time();
			triggered = true;
		} else if (triggered) {
			nrf_gpio_cfg_sense_input(BMA_INT, NRF_GPIO_PIN_NOPULL,
					NRF_GPIO_PIN_SENSE_HIGH);
			static float x = 0, y = 0, z = 0;
			uint32_t dur = bma400_get_time() - prev_time;
			shock_count++;
			uint32_t steps = bma400_get_step_count();
			bma400_get_accel(&x, &y, &z);
			triggered = false;
			mesh_tx_motion(1000 * x, 1000 * y, 1000 * z, dur, shock_count,
					steps, bma400_get_temp());

			mesh_wait_tx();
		}

	}
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt
	NVIC_EnableIRQ(GPIOTE_IRQn);
	nrf_gpio_cfg_default(STAT_LED);
	twi_stop();
	clocks_stop(); //release the hf clock
}

int main(void) {
	uint32_t err_code;

//*(volatile uint32_t *)0x40000EE4 = (*(volatile uint32_t *)0x10000258 & 0x0000004F);
	// ------------------------- Start Init -------------------------
	//err_code = NRF_LOG_INIT(NULL);
	//APP_ERROR_CHECK(err_code);
	//NRF_LOG_DEFAULT_BACKENDS_INIT();
	err_code = nrf_pwr_mgmt_init();
	APP_ERROR_CHECK(err_code);
	for (int i = 0; i < 32; i++) {
		nrf_gpio_cfg_default(i);
	}
	//NRF_LOG_DEBUG("main started");
	clocks_start();
	battery_init();

	err_code = mesh_init(app_mesh_handler, NULL); //in case of rf rx, cmd handler not required

	APP_ERROR_CHECK(err_code);

	twi_init(&m_twi);
	//NRF_LOG_DEBUG("err code %x", err_code);

	bma400_init(&m_twi);
	bma400_set_power_mode(0x02);
	bma400_set_range(0x03);
	bma400_set_odr(0x08);
	bma400_set_interrupt_on_activity();
	bma400_set_step_count();

	//NRF_GPIOTE->INTENCLR = 1 << 31;
	//NRF_GPIOTE->EVENTS_PORT = 0;
	NVIC_ClearPendingIRQ (GPIOTE_IRQn);
	nrf_gpio_cfg_sense_input(BMA_INT, NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_SENSE_HIGH);
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt
	NVIC_EnableIRQ(GPIOTE_IRQn);
	mesh_tx_reset();
	mesh_wait_tx();

	rtc_config(app_rtc_handler);

	twi_stop();
	clocks_stop(); //release the hf clock

	while (true) {
		__WFI();
		__SEV();
		__WFI();

	}

}

/*lint -restore */
