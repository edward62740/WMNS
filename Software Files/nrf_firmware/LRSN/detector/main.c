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
#include "nrf_drv_twi.h"
#include "nrf_drv_rtc.h"
#include "nrf_pwr_mgmt.h"
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
#include "uicr_user_defines.h"

//log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//drivers

#include "clocks.h"
#include "battery.h"
#include "twi.h"
#include "mesh.h"

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
uint16_t dev = 0x52;

/* Calibration parameters */
uint16_t WINDOW_LOW = 0;
uint16_t WINDOW_HIGH = 0;
uint16_t WINDOW_AVG = 0;
uint16_t WINDOW_RADIUS = 0;
uint8_t MODE = 0;

/* Global variables */
static bool react = false;
static uint32_t cycle_count = 0;
static const uint32_t period_bat = 6;
static uint32_t count = 0;
static uint8_t warn_count = 0;

volatile static uint32_t trig_th = 0;
volatile static bool trig_flag = false;
void recalibration();

void app_mesh_handler(message_t *msg) {
	NRF_LOG_INFO("app_mesh_handler()");
}

void app_rtc_handler() {

	*(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
			& 0x0000004F);
	clocks_restart();
	twi_restart();
	nrf_gpio_pin_set (STAT_LED);
	/* Measure battery voltage and transmit message if bat period */
	if ((cycle_count % period_bat) == 0) {

		uint16_t v_bat_mili = get_battery();
		mesh_tx_battery(v_bat_mili);
		mesh_wait_tx();
	} else {
		mesh_tx_alive();
		mesh_wait_tx();
	}
	trig_th = 0;
	trig_flag = false;

	nrf_gpio_pin_clear(STAT_LED);
	cycle_count++;
	twi_stop();
	clocks_stop();

}

void GPIOTE_IRQHandler(void) {
	*(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
			& 0x0000004F);
	clocks_restart();
	twi_restart();

	NVIC_DisableIRQ (GPIOTE_IRQn);
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
	if (NRF_GPIOTE->EVENTS_PORT) {
		NRF_GPIOTE->EVENTS_PORT = 0;
		nrf_gpio_cfg_output (STAT_LED); //LED
		nrf_gpio_cfg_output (CAL_LED); //LED
		nrf_gpio_cfg_input(MODE_SW, NRF_GPIO_PIN_NOPULL); //mode
		nrf_gpio_pin_set(STAT_LED);

		/* Returns to main() if CAL_SW is pressed to wake */
		if (react == true && !nrf_gpio_pin_read(CAL_SW)) {
			react = false;
			for (int j = 0; j < 7; j++) {
				nrf_gpio_pin_toggle(STAT_LED);
				nrf_gpio_pin_toggle(CAL_LED);
				nrf_delay_ms(50);
			}

			nrf_gpio_pin_clear(STAT_LED);
			nrf_gpio_pin_clear(CAL_LED);
			NVIC_SystemReset();

		}

		/* Reads VL53L1 ranging data and transmits range packet. Validates that there is no obstruction causing constant triggering
		 * If obstructed, there is a tolerance of 3 continuous triggers before triggering obstruction flag in tx every UICR_INTER_MEAS_INTV_MS * 10
		 * Obstruction flag is cleared when app_rtc_handler() is run every UICR_SLEEP_SEC */
		uint8_t range_stat = 0;
		VL53L1X_GetRangeStatus(dev, &range_stat);
		if (range_stat == 0
				&& trig_th
						< ((UICR_SLEEP_SEC * 1000) / UICR_INTER_MEAS_INTV_MS)
								- 2) {
			uint16_t dataReady;
			VL53L1X_GetDistance(dev, &dataReady);
			mesh_tx_range(dataReady, WINDOW_AVG, WINDOW_RADIUS, MODE, count,
					warn_count);
			mesh_wait_tx();
			count++;
		} else if (range_stat == 0 && trig_flag == false
				&& trig_th
						== ((UICR_SLEEP_SEC * 1000) / UICR_INTER_MEAS_INTV_MS)
								- 2) {
			uint16_t dataReady;
			VL53L1X_GetDistance(dev, &dataReady);
			mesh_tx_range(dataReady, WINDOW_AVG, WINDOW_RADIUS, MODE, count,
					warn_count);
			mesh_wait_tx();
			warn_count++;
			trig_flag = true;
			count -= (((UICR_SLEEP_SEC * 1000) / UICR_INTER_MEAS_INTV_MS) - 2);
		} else if (range_stat != 0) {
			nrf_gpio_pin_set(CAL_LED);
			VL53L1X_ClearInterrupt(dev);
			nrf_gpio_pin_clear(CAL_LED);
		}

		trig_th++;

		/* Long press CAL_SW to return to main() to be powered off */
		if (!nrf_gpio_pin_read(CAL_SW)) {
			NVIC_SystemReset();
		}
	}
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	VL53L1X_ClearInterrupt(dev);
	NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt
	NVIC_EnableIRQ(GPIOTE_IRQn);

	nrf_gpio_pin_clear (STAT_LED);
	twi_stop();
	clocks_stop(); //release the hf clock

}

void calibration() {
	uint32_t WINDOW_SUM = 0;

	for (int i = 0; i < 20; i++) {
		uint8_t range_stat = 0;
		uint16_t dataReady;
		VL53L1X_GetRangeStatus(dev, &range_stat);
		VL53L1X_GetDistance(dev, &dataReady);
		VL53L1X_ClearInterrupt(dev);
		if (range_stat == 0) {
			WINDOW_SUM += dataReady;
		}
		nrf_gpio_pin_toggle (CAL_LED);
		nrf_delay_ms(1);

		while (nrf_gpio_pin_read (VL53L1_INT)) {
		}
	}
	WINDOW_AVG = WINDOW_SUM / 20;
	WINDOW_RADIUS = UICR_OUT_OF_BOUNDS_TOL * WINDOW_AVG;
	WINDOW_LOW = WINDOW_AVG - WINDOW_RADIUS;
	WINDOW_HIGH = WINDOW_AVG + WINDOW_RADIUS;

	VL53L1X_SetDistanceThreshold(dev, WINDOW_LOW, WINDOW_HIGH, 2, 0);

	mesh_tx_range(WINDOW_AVG, WINDOW_AVG, WINDOW_RADIUS, 4, count, 0);
	mesh_wait_tx();
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
	nrf_gpio_cfg_output (STAT_LED); //LED
	nrf_gpio_cfg_output (CAL_LED); //LED
	nrf_gpio_cfg_output (VL53L1_SD); //shutdown
	nrf_gpio_cfg_input(MODE_SW, NRF_GPIO_PIN_NOPULL); //mode
	nrf_gpio_cfg_input(CAL_SW, NRF_GPIO_PIN_NOPULL); //cal

	nrf_gpio_pin_set(VL53L1_SD);
	nrf_gpio_pin_set(STAT_LED);
	nrf_gpio_pin_set(CAL_LED);

	mesh_tx_reset();
	mesh_wait_tx();
	nrf_delay_ms(500);

	/* If CAL_SW is pressed while main() is running (i.e. shutdown requested), set interrupt on CAL_SW for wake up and enter shutdown */
	if (!nrf_gpio_pin_read(CAL_SW)) {
		nrf_gpio_pin_clear(CAL_LED);
		nrf_gpio_pin_clear(STAT_LED);
		nrf_delay_ms(700);
		react = true;
		NVIC_ClearPendingIRQ (GPIOTE_IRQn);
		NRF_GPIOTE->EVENTS_PORT = 0;
		nrf_gpio_cfg_sense_input(CAL_SW, NRF_GPIO_PIN_NOPULL,
				NRF_GPIO_PIN_SENSE_LOW);
		NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
		NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt
		NVIC_EnableIRQ(GPIOTE_IRQn);
		nrf_gpio_pin_set(CAL_LED);

		for (int j = 0; j < 7; j++) {
			nrf_gpio_pin_toggle(STAT_LED);
			nrf_gpio_pin_toggle(CAL_LED);
			nrf_delay_ms(50);
		}

		nrf_gpio_pin_clear(STAT_LED);
		nrf_gpio_pin_clear(CAL_LED);
		rtc_config(app_rtc_handler);
		twi_stop();
		clocks_stop(); //release the hf clock
		//NRF_TIMER0-> TASKS_SHUTDOWN = 1;
		while (true) {
			nrf_pwr_mgmt_run();
		}
	}

	/* Normal initialization if CAL_SW is not pressed  */
	VL53L1(&m_twi);
	VL53L1X_SensorInit(dev);
	VL53L1X_StartTemperatureUpdate(dev);
	VL53L1X_SetInterMeasurementInMs(dev, 250);
	VL53L1X_SetTimingBudgetInMs(dev, 200);
	VL53L1X_SetDistanceThreshold(dev, 0, 1, 1, 1);
	VL53L1X_SetInterruptPolarity(dev, 0);
	VL53L1X_StartRanging(dev);
	calibration();
	VL53L1X_SetInterMeasurementInMs(dev, UICR_INTER_MEAS_INTV_MS);
	if (nrf_gpio_pin_read (MODE_SW)) //read distance mode
	{
		VL53L1X_SetDistanceMode(dev, 2); //long
		VL53L1X_SetTimingBudgetInMs(dev, UICR_TIMING_BUDGET_MS);
		MODE = 2;
	} else {
		VL53L1X_SetDistanceMode(dev, 1); //short
		VL53L1X_SetTimingBudgetInMs(dev, UICR_TIMING_BUDGET_MS);
		MODE = 1;
	}

	VL53L1X_StartRanging(dev);

	NVIC_ClearPendingIRQ (GPIOTE_IRQn);
	NRF_GPIOTE->EVENTS_PORT = 0;
	nrf_gpio_cfg_sense_input(VL53L1_INT, NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_input(CAL_SW, NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_SENSE_LOW);
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt
	NVIC_EnableIRQ(GPIOTE_IRQn);

	rtc_config(app_rtc_handler);
	twi_stop();
	clocks_stop(); //release the hf clock

	nrf_gpio_pin_clear(CAL_LED);
	nrf_gpio_pin_clear(STAT_LED);
	while (true) {
		nrf_pwr_mgmt_run();
	}
}

