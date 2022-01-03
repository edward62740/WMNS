#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "si7021.h"
#include "math.h"

static const uint8_t address = 0x40;
static const nrf_drv_twi_t *p_twi = NULL;

uint8_t si7021_write_reg(uint8_t reg, uint8_t value) {
	uint8_t res[3];
	res[0] = reg;
	res[1] = value;

	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, res, 2, false);
	APP_ERROR_CHECK(err_code);
	return 0;
}

uint16_t si7021_read_reg(uint8_t reg) {
	uint8_t res[2];

	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(p_twi, address, res, 2);
	APP_ERROR_CHECK(err_code);

	return (uint16_t)((res[1] << 8) | res[0]);
}

void si7021_init(const nrf_drv_twi_t *l_twi) {
	p_twi = l_twi;
}

void si7021_start_temperature_no_hold_master() {
	uint8_t cmd = SI7021_READPREVTEMP_CMD;
	nrf_drv_twi_tx(p_twi, address, &cmd, 1, false);
}

bool si7021_fetch_temperature_no_hold_master(int32_t *temp) {
	int32_t val;
	uint8_t res[3]; //Array to hold returned data
	if (nrf_drv_twi_rx(p_twi, address, res, 2)
			== NRF_ERROR_DRV_TWI_ERR_ANACK) {
		return false;
	} else {
		val = (int32_t)((res[0] << 8) | (res[1] & 0xFC));
		*temp = (((val * 21965L) >> 13) - 46850);

		return true;
	}
}

void si7021_start_humidity_no_hold_master() {
	uint8_t cmd = SI7021_MEASRH_NOHOLD_CMD;
	nrf_drv_twi_tx(p_twi, address, &cmd, 1, false);

}

bool si7021_fetch_humidity_no_hold_master(uint32_t *hum) {
	uint32_t val;

	uint8_t res[2]; //Array to hold returned data
	if (nrf_drv_twi_rx(p_twi, address, res, 2)
			== NRF_ERROR_DRV_TWI_ERR_ANACK) {
		return false;
	} else {
		val = (uint32_t)((res[0] << 8) | (res[1] & 0xFC));
		*hum = (uint32_t)((((val *  15625L) >> 13) - 6000));
		//hum = val;
		return true;
	}

}

void si7021_reset() {
	uint8_t cmd = SI7021_MEASTEMP_NOHOLD_CMD;
	nrf_drv_twi_tx(p_twi, address, &cmd, 1, false);
}
