/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "battery.h"

#include "sdk_common.h"
#include "boards.h"

#include "nrf_drv_saadc.h"

#include "nrf_log.h"

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  600  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define NRF52832_MODULE_DROP_MILLIVOLTS 70  
#define ADC_RES_10BIT                  1024 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   6    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

#define CO2SN_VOLTAGE_DIVIDER_MULTIPLIER 1.3

static nrf_saadc_value_t adc_buf;        //!< Buffer used for storing ADC value.
static uint16_t m_batt_lvl_in_milli_volts = 9000; //!< Current battery level.
static bool status_done = false;

/**@brief Function handling events from 'nrf_drv_saadc.c'.
 *
 * @param[in] p_evt SAADC event.
 */
static void saadc_event_handler(nrf_drv_saadc_evt_t const *p_evt) {
	if (p_evt->type == NRF_DRV_SAADC_EVT_DONE) {
		nrf_saadc_value_t adc_result;

		adc_result = p_evt->data.done.p_buffer[0];

#if defined (CO2SN)
		m_batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
#else
		m_batt_lvl_in_milli_volts =
		ADC_RESULT_IN_MILLI_VOLTS(adc_result) + NRF52832_MODULE_DROP_MILLIVOLTS;
#endif
	}
	status_done = true;
}

//Start conversion can be called by the user to make sure the measure has just taken place
//This function is though optionally called, as a simpler shifted sample polling is possible
//given that get_battery() also starts a new conversion
void start_conversion() {
	ret_code_t err_code;

	status_done = false;
	err_code = nrf_drv_saadc_buffer_convert(&adc_buf, 1);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_sample();
	APP_ERROR_CHECK(err_code);
}

void battery_init(void) {
	ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);

	APP_ERROR_CHECK(err_code);

#if defined (CO2SN)
	nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(
			NRF_SAADC_INPUT_AIN3);
#else
	nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(
			NRF_SAADC_INPUT_VDD);
#endif
	err_code = nrf_drv_saadc_channel_init(0, &config);
	APP_ERROR_CHECK(err_code);
	start_conversion();
}

//Gives back the last measure, keeps waiting only if the last started conversion is still ongoing
uint16_t get_battery() {
	//make sure the last started conversion is done before returning
	while (!status_done)
		;

	//return the latest sampled value
#if defined (CO2SN)
	uint16_t vbatt = m_batt_lvl_in_milli_volts * CO2SN_VOLTAGE_DIVIDER_MULTIPLIER;
#else
	uint16_t vbatt = m_batt_lvl_in_milli_volts;
#endif

	//check if possible to start a new conversion
	if (!nrf_drv_saadc_is_busy()) {
		start_conversion();
	}

	return vbatt;
}
