#include "ltr390.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "math.h"

static const uint8_t address = 0x53;
static const nrf_drv_twi_t *p_twi = NULL;

void ltr390_write_reg(uint8_t reg, uint8_t data) {
	uint8_t res[2];
	res[0] = reg;
	res[1] = data;

	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, res, 2, false);
	APP_ERROR_CHECK(err_code);
}

uint8_t ltr390_read_reg(uint8_t reg) {

	uint8_t res;

	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(p_twi, address, &res, 1);
	APP_ERROR_CHECK(err_code);

	return res;
}

void ltr390_read_reg_burst(uint8_t reg, uint8_t length, uint8_t *data) {


	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(p_twi, address, data, length);
	APP_ERROR_CHECK(err_code);

}

void ltr390_write_reg_burst(uint8_t reg, uint8_t length, uint8_t *data) {

	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(p_twi, address, data, length, false);
	APP_ERROR_CHECK(err_code);

}

bool ltr390_init(const nrf_drv_twi_t *l_twi) {

	p_twi = l_twi;
	if (ltr390_read_reg(LTR390_PART_ID) != 0xB2) {
		return false;
	}

	return true;
}

/*!
 *  @brief  Perform a soft reset with 10ms delay.
 *  @returns True on success (reset bit was cleared post-write)
 */
bool ltr390_reset(void) {

	uint8_t data;

	data = ltr390_read_reg(LTR390_MAIN_CTRL);
	data |= 0x10;
	ltr390_write_reg(LTR390_MAIN_CTRL, data);

	nrf_delay_ms(10);
	if (ltr390_read_reg(LTR390_MAIN_CTRL) & 0x10) {
		return false;
	}
	return true;
}

/*!
 *  @brief  Checks if new data is available in data register
 *  @returns True on new data available
 */
bool ltr390_newDataAvailable(void) {

	if ((ltr390_read_reg(LTR390_MAIN_STATUS) | 0x8)) {
		return true;
	}
	return false;
}

/*!
 *  @brief  Read 3-bytes out of ambient data register, does not check if data is
 * new!
 *  @returns Up to 20 bits, right shifted into a 32 bit int
 */
uint32_t ltr390_readALS(void) {

	uint8_t data[3];
	ltr390_read_reg_burst(LTR390_ALSDATA, 3, data);


       	return (uint32_t)((data[2] << 16) | (data[1] << 8) | (data[0]));
}

/*!
 *  @brief  Read 3-bytes out of UV data register, does not check if data is new!
 *  @returns Up to 20 bits, right shifted into a 32 bit int
 */
uint32_t ltr390_readUVS(void) {

	uint8_t data[3];
	ltr390_read_reg_burst(0x10, 3, data);

       	return (uint32_t)((data[2] << 16) | (data[1] << 8) | (data[0]));
}

void ltr390_enable(void) {
	uint8_t data;
	data = ltr390_read_reg(LTR390_MAIN_CTRL) | 0x2;
	ltr390_write_reg(LTR390_MAIN_CTRL, data);
}
void ltr390_disable(void) {
	uint8_t data;
	data = ltr390_read_reg(LTR390_MAIN_CTRL) ^ 0x2;
	ltr390_write_reg(LTR390_MAIN_CTRL, data);
}

/*!
 *  @brief  Read the enabled-bit from the sensor
 *  @returns True if enabled
 */
bool ltr390_enabled(void) {

	if (ltr390_read_reg(LTR390_MAIN_CTRL) & 0x2) {
		return false;
	}
	return true;
}

/*!
 *  @brief  Set the sensor mode to EITHER ambient (LTR390_MODE_ALS) or UV
 * (LTR390_MODE_UVS)
 *  @param  mode The desired mode - LTR390_MODE_UVS or LTR390_MODE_ALS
 */
void ltr390_setMode(ltr390_mode_t mode) {

	uint8_t data;
	data = ltr390_read_reg(LTR390_MAIN_CTRL);

	if (LTR390_MODE_ALS) {
		data ^= 0x8;
		ltr390_write_reg(LTR390_MAIN_CTRL, data);
	}
	if (LTR390_MODE_UVS) {
		data |= 0x8;
		ltr390_write_reg(LTR390_MAIN_CTRL, data);
	}
}

/*!
 *  @brief  get the sensor's modeca
 *  @returns The current mode - LTR390_MODE_UVS or LTR390_MODE_ALS
 */
uint8_t ltr390_getMode(void) {
	return ((ltr390_read_reg(LTR390_MAIN_CTRL) & 0x8) >> 3) ? 0 : 1;
}

/*!
 *  @brief  Set the sensor gain
 *  @param  gain The desired gain: LTR390_GAIN_1, LTR390_GAIN_3, LTR390_GAIN_6
 *  LTR390_GAIN_9 or LTR390_GAIN_18
 */
void ltr390_setGain(ltr390_gain_t gain) {
	uint8_t data;
	data = ltr390_read_reg(LTR390_GAIN) & 0x0;
	ltr390_write_reg(LTR390_GAIN, (data | gain));
}

/*!
 *  @brief  Get the sensor's gain
 *  @returns gain The current gain: LTR390_GAIN_1, LTR390_GAIN_3, LTR390_GAIN_6
 *  LTR390_GAIN_9 or LTR390_GAIN_18
 */
uint8_t ltr390_getGain(void) {

	return (uint8_t)(ltr390_read_reg(LTR390_GAIN) & 0x7);
}

/*!
 *  @brief  Set the sensor resolution. Higher resolutions take longer to read!
 *  @param  res The desired resolution: LTR390_RESOLUTION_13BIT,
 *  LTR390_RESOLUTION_16BIT, LTR390_RESOLUTION_17BIT, LTR390_RESOLUTION_18BIT,
 *  LTR390_RESOLUTION_19BIT or LTR390_RESOLUTION_20BIT
 */
void ltr390_setResolution(ltr390_resolution_t res) {
	uint8_t data;
	data = ltr390_read_reg(LTR390_MEAS_RATE) & 0x70;
	ltr390_write_reg(LTR390_MEAS_RATE, (data | (res << 4)));
}

/*!
 *  @brief  Get the sensor's resolution
 *  @returns The current resolution: LTR390_RESOLUTION_13BIT,
 *  LTR390_RESOLUTION_16BIT, LTR390_RESOLUTION_17BIT, LTR390_RESOLUTION_18BIT,
 *  LTR390_RESOLUTION_19BIT or LTR390_RESOLUTION_20BIT
 */
uint8_t ltr390_getResolution(void) {
	return (uint8_t)((ltr390_read_reg(LTR390_MEAS_RATE) & 0x70) >> 4);
}


