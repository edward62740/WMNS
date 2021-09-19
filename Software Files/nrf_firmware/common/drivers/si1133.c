#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "si1133.h"
#include "math.h"

static const uint8_t address = 0x55;
static const nrf_drv_twi_t *p_twi = NULL;

/***************************************************************************//**
 * @brief
 *    Coefficients for lux calculation
 ******************************************************************************/
static si1133_LuxCoeff_TypeDef lk = { { { 0, 209 }, /**< coeff_high[0]   */
{ 1665, 93 }, /**< coeff_high[1]   */
{ 2064, 65 }, /**< coeff_high[2]   */
{ -2671, 234 } }, /**< coeff_high[3]   */
{ { 0, 0 }, /**< coeff_low[0]    */
{ 1921, 29053 }, /**< coeff_low[1]    */
{ -1022, 36363 }, /**< coeff_low[2]    */
{ 2320, 20789 }, /**< coeff_low[3]    */
{ -367, 57909 }, /**< coeff_low[4]    */
{ -1774, 38240 }, /**< coeff_low[5]    */
{ -608, 46775 }, /**< coeff_low[6]    */
{ -1503, 51831 }, /**< coeff_low[7]    */
{ -1886, 58928 } } /**< coeff_low[8]    */
};

/***************************************************************************//**
 * @brief
 *    Coefficients for UV index calculation
 ******************************************************************************/
static si1133_Coeff_TypeDef uk[2] = { { 1281, 30902 }, /**< coeff[0]        */
{ -638, 46301 } /**< coeff[1]        */
};

static int32_t si1133_calcPolyInner(int32_t input, int8_t fraction,
		uint16_t mag, int8_t shift);
static int32_t si1133_calcEvalPoly(int32_t x, int32_t y, uint8_t input_fraction,
		uint8_t output_fraction, uint8_t num_coeff, si1133_Coeff_TypeDef *kp);

uint32_t si1133_registerRead(uint8_t reg, uint8_t *data) {

	uint32_t retval;
	retval = si1133_OK;

	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(p_twi, address, data, 1);
	APP_ERROR_CHECK(err_code);
	return retval;
}

uint32_t si1133_registerWrite(uint8_t reg, uint8_t data) {
	uint32_t retval;
	retval = si1133_OK;

	uint8_t buffer[2];
	buffer[0] = reg;
	buffer[1] = data;

	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, buffer, 2, false);
	APP_ERROR_CHECK(err_code);
	return retval;
}

uint32_t si1133_registerBlockWrite(uint8_t reg, uint8_t length, uint8_t *data) {

	uint32_t retval;
	retval = si1133_OK;

	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(p_twi, address, data, length, false);
	APP_ERROR_CHECK(err_code);
	return retval;
}

uint32_t si1133_registerBlockRead(uint8_t reg, uint8_t length, uint8_t *data) {

	uint32_t retval;
	retval = si1133_OK;

	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(p_twi, address, data, length);
	APP_ERROR_CHECK(err_code);
	return retval;
}

/***************************************************************************//**
 * @brief
 *    Waits until the si1133 is sleeping before proceeding
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
static uint32_t si1133_waitUntilSleep(void) {
	uint32_t ret;
	uint8_t response;
	uint8_t count = 0;
	uint32_t retval;

	retval = si1133_OK;

	/* This loops until the si1133 is known to be in its sleep state  */
	/* or if an i2c error occurs                                      */
	while (count < 5) {
		ret = si1133_registerRead(si1133_REG_RESPONSE0, &response);
		if ((response & si1133_RSP0_CHIPSTAT_MASK) == si1133_RSP0_SLEEP) {
			break;
		}

		if (ret != si1133_OK) {
			retval = si1133_ERROR_SLEEP_FAILED;
			break;
		}

		count++;
	}

	return retval;
}

/***************************************************************************//**
 * @brief
 *    Resets the si1133
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_reset(void) {
	uint32_t retval;

	/* Do not access the si1133 earlier than 25 ms from power-up */
	nrf_delay_ms(30);

	/* Perform the Reset Command */
	retval = si1133_registerWrite(si1133_REG_COMMAND, si1133_CMD_RESET);

	/* Delay for 10 ms. This delay is needed to allow the si1133   */
	/* to perform internal reset sequence.                         */
	nrf_delay_ms(10);

	return retval;
}

/***************************************************************************//**
 * @brief
 *    Helper function to send a command to the si1133
 *
 * @param[in] command
 *    The command to send to the sensor
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
static uint32_t si1133_sendCmd(uint8_t command) {
	uint8_t response;
	uint8_t response_stored;
	uint8_t count = 0;
	uint32_t ret;

	/* Get the response register contents */
	ret = si1133_registerRead(si1133_REG_RESPONSE0, &response_stored);
	if (ret != si1133_OK) {
		return ret;
	}

	response_stored = response_stored & si1133_RSP0_COUNTER_MASK;

	/* Double-check the response register is consistent */
	while (count < 5) {
		ret = si1133_waitUntilSleep();
		if (ret != si1133_OK) {
			return ret;
		}
		/* Skip if the command is RESET COMMAND COUNTER */
		if (command == si1133_CMD_RESET_CMD_CTR) {
			break;
		}

		ret = si1133_registerRead(si1133_REG_RESPONSE0, &response);

		if ((response & si1133_RSP0_COUNTER_MASK) == response_stored) {
			break;
		} else {
			if (ret != si1133_OK) {
				return ret;
			} else {
				response_stored = response & si1133_RSP0_COUNTER_MASK;
			}
		}

		count++;
	}

	/* Send the command */
	ret = si1133_registerWrite(si1133_REG_COMMAND, command);
	if (ret != si1133_OK) {
		return ret;
	}

	count = 0;
	/* Expect a change in the response register */
	while (count < 5) {
		/* Skip if the command is RESET COMMAND COUNTER */
		if (command == si1133_CMD_RESET_CMD_CTR) {
			break;
		}

		ret = si1133_registerRead(si1133_REG_RESPONSE0, &response);
		if ((response & si1133_RSP0_COUNTER_MASK) != response_stored) {
			break;
		} else {
			if (ret != si1133_OK) {
				return ret;
			}
		}

		count++;
	}

	return si1133_OK;
}

/***************************************************************************//**
 * @brief
 *    Sends a RESET COMMAND COUNTER command to the si1133
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_resetCmdCtr(void) {
	return si1133_sendCmd(si1133_CMD_RESET_CMD_CTR);
}

/***************************************************************************//**
 * @brief
 *    Sends a FORCE command to the si1133
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_measurementForce(void) {
	return si1133_sendCmd(si1133_CMD_FORCE_CH);
}

/***************************************************************************//**
 * @brief
 *    Sends a START command to the si1133
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_measurementStart(void) {
	return si1133_sendCmd(si1133_CMD_START);
}

/***************************************************************************//**
 * @brief
 *    Reads a parameter from the si1133
 *
 * @param[in] address
 *    The address of the parameter.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_paramRead(uint8_t address) {
	uint8_t retval;
	uint8_t cmd;

	cmd = 0x40 + (address & 0x3F);

	retval = si1133_sendCmd(cmd);
	if (retval != si1133_OK) {
		return retval;
	}

	si1133_registerRead(si1133_REG_RESPONSE1, &retval);

	return retval;
}

/***************************************************************************//**
 * @brief
 *    Writes a byte to an si1133 Parameter
 *
 * @param[in] address
 *    The parameter address
 *
 * @param[in] value
 *    The byte value to be written to the si1133 parameter
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 *si1133_CMD_START
 * @note
 *    This function ensures that the si1133 is idle and ready to
 *    receive a command before writing the parameter. Furthermore,
 *    command completion is checked. If setting parameter is not done
 *    properly, no measurements will occur. This is the most common
 *    error. It is highly recommended that host code make use of this
 *    function.
 ******************************************************************************/
uint32_t si1133_paramSet(uint8_t address, uint8_t value) {
	uint32_t retval;
	uint8_t buffer[2];
	uint8_t response_stored;
	uint8_t response;
	uint8_t count;

	retval = si1133_waitUntilSleep();
	if (retval != si1133_OK) {
		return retval;
	}

	si1133_registerRead(si1133_REG_RESPONSE0, &response_stored);
	response_stored &= si1133_RSP0_COUNTER_MASK;

	buffer[0] = value;
	buffer[1] = 0x80 + (address & 0x3F);

	retval = si1133_registerBlockWrite(si1133_REG_HOSTIN0, 2,
			(uint8_t*) buffer);
	if (retval != si1133_OK) {
		return retval;
	}

	/* Wait for command to finish */
	count = 0;
	/* Expect a change in the response register */
	while (count < 5) {
		retval = si1133_registerRead(si1133_REG_RESPONSE0, &response);
		if ((response & si1133_RSP0_COUNTER_MASK) != response_stored) {
			break;
		} else {
			if (retval != si1133_OK) {
				return retval;
			}
		}

		count++;
	}

	return si1133_OK;
}

/***************************************************************************//**
 * @brief
 *    Sends a PAUSE command to the si1133
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_measurementPause(void) {
	return si1133_sendCmd(si1133_CMD_PAUSE_CH);
}

/**************************************************************************//**
 * @brief
 *    Initializes the si1133 chip
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 *****************************************************************************/
uint32_t si1133_init(const nrf_drv_twi_t *l_twi) {
	uint32_t retval;

	p_twi = l_twi;

	/* Allow some time for the part to power up */
	nrf_delay_ms(5);

	retval = si1133_reset();
	
	
	nrf_delay_ms(10);

	
	//retval += si1133_registerWrite(0x0B, 0x01);
	retval += si1133_paramSet(si1133_PARAM_CH_LIST, 0x01);

	retval += si1133_paramSet(si1133_PARAM_ADCCONFIG0, 0x78);  //uv 24.4us
	retval += si1133_paramSet(si1133_PARAM_ADCSENS0, 0x09);
	retval += si1133_paramSet(si1133_PARAM_ADCPOST0, 0x40);
	
/*	retval += si1133_paramSet(si1133_PARAM_MEASCONFIG0, 0x40);
	retval += si1133_paramSet(si1133_PARAM_MEASRATE_L, 0x7d);
	retval += si1133_paramSet(si1133_PARAM_MEASRATE_H, 0x00);
	retval += si1133_paramSet(si1133_PARAM_MEASCOUNT0, 0x01);

	retval += si1133_paramSet(si1133_PARAM_ADCCONFIG1, 0x4d); //large white 196us
	retval += si1133_paramSet(si1133_PARAM_ADCSENS1, 0xe1);
	retval += si1133_paramSet(si1133_PARAM_ADCPOST1, 0x40);

	retval += si1133_paramSet(si1133_PARAM_ADCCONFIG2, 0x41);  //medium ir 196us
	retval += si1133_paramSet(si1133_PARAM_ADCSENS2, 0xe1);
	retval += si1133_paramSet(si1133_PARAM_ADCPOST2, 0x50);

	retval += si1133_paramSet(si1133_PARAM_ADCCONFIG3, 0x4d);
	retval += si1133_paramSet(si1133_PARAM_ADCSENS3, 0x87);
	retval += si1133_paramSet(si1133_PARAM_ADCPOST3, 0x40);
*/
	retval += si1133_registerWrite(si1133_REG_IRQ_ENABLE, 0x01);
	return retval;
}

/***************************************************************************//**
 * @brief
 *    Stops the measurements on all channel and waits until the chip
 *    goes to sleep state.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_deInit(void) {
	uint32_t retval;

	retval = si1133_paramSet(si1133_PARAM_CH_LIST, 0x3f);
	retval += si1133_measurementPause();
	retval += si1133_waitUntilSleep();

	return retval;
}

/***************************************************************************//**
 * @brief
 *    Read samples from the si1133 chip
 *
 * @param[out] samples
 *    Retrieves interrupt status and measurement data for channel 0..3 and
 *    converts the data to int32_t format
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_readMeasurement(si1133_Samples_TypeDef *samples) {
	uint8_t buffer[13];
	uint32_t retval;

	retval = si1133_registerBlockRead(si1133_REG_IRQ_STATUS, 13, buffer);

	samples->irq_status = buffer[0];

	samples->ch0 = buffer[1] << 16;
	samples->ch0 |= buffer[2] << 8;
	samples->ch0 |= buffer[3];
	if (samples->ch0 & 0x800000) {
		samples->ch0 |= 0xFF000000;
	}

	samples->ch1 = buffer[4] << 16;
	samples->ch1 |= buffer[5] << 8;
	samples->ch1 |= buffer[6];
	if (samples->ch1 & 0x800000) {
		samples->ch1 |= 0xFF000000;
	}

	samples->ch2 = buffer[7] << 16;
	samples->ch2 |= buffer[8] << 8;
	samples->ch2 |= buffer[9];
	if (samples->ch2 & 0x800000) {
		samples->ch2 |= 0xFF000000;
	}

	samples->ch3 = buffer[10] << 16;
	samples->ch3 |= buffer[11] << 8;
	samples->ch3 |= buffer[12];
	if (samples->ch3 & 0x800000) {
		samples->ch3 |= 0xFF000000;
	}

	return retval;
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/***************************************************************************//**
 * @brief
 *    Helper function to polynomial calculations
 ******************************************************************************/
int32_t si1133_calcPolyInner(int32_t input, int8_t fraction, uint16_t mag,
		int8_t shift) {
	int32_t value;

	if (shift < 0) {
		value = ((input << fraction) / mag) >> -shift;
	} else {
		value = ((input << fraction) / mag) << shift;
	}

	return value;
}

/***************************************************************************//**
 * @brief
 *    Polynomial evaluation function to calculate lux and UV index
 *
 * @return
 *    Scaled results
 ******************************************************************************/
int32_t si1133_calcEvalPoly(int32_t x, int32_t y, uint8_t input_fraction,
		uint8_t output_fraction, uint8_t num_coeff, si1133_Coeff_TypeDef *kp) {
	uint8_t info, x_order, y_order, counter;
	int8_t sign, shift;
	uint16_t mag;
	int32_t output = 0, x1, x2, y1, y2;

	for (counter = 0; counter < num_coeff; counter++) {
		info = kp->info;
		x_order = get_x_order(info);
		y_order = get_y_order(info);

		shift = ((uint16_t) kp->info & 0xff00) >> 8;
		shift ^= 0x00ff;
		shift += 1;
		shift = -shift;

		mag = kp->mag;

		if (get_sign(info)) {
			sign = -1;
		} else {
			sign = 1;
		}

		if ((x_order == 0) && (y_order == 0)) {
			output += sign * mag << output_fraction;
		} else {
			if (x_order > 0) {
				x1 = si1133_calcPolyInner(x, input_fraction, mag, shift);
				if (x_order > 1) {
					x2 = si1133_calcPolyInner(x, input_fraction, mag, shift);
				} else {
					x2 = 1;
				}
			} else {
				x1 = 1;
				x2 = 1;
			}

			if (y_order > 0) {
				y1 = si1133_calcPolyInner(y, input_fraction, mag, shift);
				if (y_order > 1) {
					y2 = si1133_calcPolyInner(y, input_fraction, mag, shift);
				} else {
					y2 = 1;
				}
			} else {
				y1 = 1;
				y2 = 1;
			}

			output += sign * x1 * x2 * y1 * y2;
		}

		kp++;
	}

	if (output < 0) {
		output = -output;
	}

	return output;
}

/** @endcond */

/***************************************************************************//**
 * @brief
 *    Compute UV index
 *
 * @param[in] uv
 *    UV sensor raw data
 *
 * @param[in] uk
 *    UV calculation coefficients
 *
 * @return
 *    UV index scaled by UV_OUPTUT_FRACTION
 ******************************************************************************/
int32_t si1133_getUv(int32_t uv, si1133_Coeff_TypeDef *uk) {
	int32_t uvi;

	uvi = si1133_calcEvalPoly(0, uv, UV_INPUT_FRACTION, UV_OUTPUT_FRACTION,
			UV_NUMCOEFF, uk);

	return uvi;
}

/***************************************************************************//**
 * @brief
 *    Compute lux value
 *
 * @param[in] vis_high
 *    Visible light sensor raw data
 *
 * @param[in] vis_low
 *    Visible light sensor raw data
 *
 * @param[in] ir
 *    Infrared sensor raw data
 *
 * @param[in] lk
 *    Lux calculation coefficients
 *
 * @return
 *    Lux value scaled by LUX_OUPTUT_FRACTION
 ******************************************************************************/
int32_t si1133_getLux(int32_t vis_high, int32_t vis_low, int32_t ir,
		si1133_LuxCoeff_TypeDef *lk) {
	int32_t lux;

	if ((vis_high > ADC_THRESHOLD) || (ir > ADC_THRESHOLD)) {
		lux = si1133_calcEvalPoly(vis_high, ir, INPUT_FRACTION_HIGH,
				LUX_OUTPUT_FRACTION, NUMCOEFF_HIGH, &(lk->coeff_high[0]));
	} else {
		lux = si1133_calcEvalPoly(vis_low, ir, INPUT_FRACTION_LOW,
				LUX_OUTPUT_FRACTION, NUMCOEFF_LOW, &(lk->coeff_low[0]));
	}

	return lux;
}


/***************************************************************************//**
 * @brief
 *    Retrieve the sample values from the chip and convert them
 *    to lux and UV index values
 *
 * @param[out] lux
 *    The measured ambient light illuminace in lux
 *
 * @param[out] uvi
 *    UV index
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_getMeasurement(float *lux, float *uvi) {
	si1133_Samples_TypeDef samples;

	uint8_t response;
	uint32_t retval = 0;

	retval += si1133_registerRead(si1133_REG_IRQ_STATUS, &response);
	if (response != 0x0F) {

		/* Get the results */
		retval = si1133_readMeasurement(&samples);

		/* Convert the readings to lux */
		*lux = (float) si1133_getLux(samples.ch1, samples.ch3, samples.ch2,
				&lk);
		*lux = *lux / (1 << LUX_OUTPUT_FRACTION);

		/* Convert the readings to UV index */
		*uvi = (float) si1133_getUv(samples.ch0, uk);
		*uvi = *uvi / (1 << UV_OUTPUT_FRACTION);
	}
	return retval;
}

/***************************************************************************//**
 * @brief
 *    Reads the interrupt status register of the device
 *
 * @param[out] irqStatus
 *    The contentof the IRQ status register
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_getIrqStatus(uint8_t *irqStatus) {
	uint32_t retval;

	/* Read the IRQ status register */
	retval = si1133_registerRead(si1133_REG_IRQ_STATUS, irqStatus);

	return retval;
}


/***************************************************************************//**
 * @brief
 *    Reads Hardware ID from the si1133 sensor
 *
 * @param[out] hardwareID
 *    The Hardware ID of the chip (should be 0x33)
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t si1133_getHardwareID(uint8_t *hardwareID) {
	uint32_t retval;

	retval = si1133_registerRead(si1133_REG_PART_ID, hardwareID);

	return retval;
}

