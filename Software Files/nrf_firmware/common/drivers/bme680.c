#include "bme680.h"

#define NRF_LOG_MODULE_NAME bme

#if (BME_CONFIG_LOG_ENABLED == 1)
#define NRF_LOG_LEVEL BME_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR BME_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BME_CONFIG_DEBUG_COLOR
#else //BME_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BME_CONFIG_LOG_ENABLED

#include "nrf_log.h"
#include "nrf_delay.h"
NRF_LOG_MODULE_REGISTER();

struct bme680_driver bme680; /* global instance */

static const uint8_t address = 0x76;

static const nrf_drv_twi_t *p_twi = NULL;



void bme680_read_reg(uint8_t start, uint8_t length, uint8_t* buffer)
{
  ret_code_t err_code;
  err_code = nrf_drv_twi_tx(p_twi, address, &start, 1,true);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_twi_rx(p_twi, address, buffer, length);
  APP_ERROR_CHECK(err_code);

}

void bme680_write_reg(uint8_t reg, uint8_t value)
{
  ret_code_t err_code;
  uint8_t data[2];
  data[0] = reg;
  data[1] = value;
  err_code = nrf_drv_twi_tx(p_twi, address, data, 2,false);
  APP_ERROR_CHECK(err_code);

}

void bme680_init(const nrf_drv_twi_t *l_twi)
{

  p_twi = l_twi;

  uint8_t buffer1[23];
  uint8_t buffer2[14];
  uint8_t buffer3[5];
  uint8_t id;

  bme680_read_reg(bme680REG_ID, 23, &id);



  bme680_read_reg(0x8A, 23, buffer1);
  bme680_read_reg(0xE1, 14, buffer2);
  bme680_read_reg(0x00, 5, buffer3);

  // load calibration data...


  bme680.cp.par_t1 = (uint16_t)(bme680_CONCAT_BYTES(buffer2[9], buffer2[8]));
  bme680.cp.par_t2 = (uint16_t)(bme680_CONCAT_BYTES(buffer1[1], buffer1[0]));
  bme680.cp.par_t3 = (uint8_t)(buffer1[2]);

  bme680.cp.par_p1 = (uint16_t)(bme680_CONCAT_BYTES(buffer1[5], buffer1[4]));
  bme680.cp.par_p2= (int16_t)(bme680_CONCAT_BYTES(buffer1[7], buffer1[6]));
  bme680.cp.par_p3 = (int8_t)buffer1[8];
  bme680.cp.par_p4 = (int16_t)(bme680_CONCAT_BYTES(buffer1[11], buffer1[10]));
  bme680.cp.par_p5 = (int16_t)(bme680_CONCAT_BYTES(buffer1[13], buffer1[12]));
  bme680.cp.par_p6 = (int8_t)(buffer1[15]);
  bme680.cp.par_p7 = (int8_t)(buffer1[14]);
  bme680.cp.par_p8 = (int16_t)(bme680_CONCAT_BYTES(buffer1[19], buffer1[18]));
  bme680.cp.par_p9 = (int16_t)(bme680_CONCAT_BYTES(buffer1[21], buffer1[20]));
  bme680.cp.par_p10 = (uint8_t)(buffer1[22]);

  bme680.cp.par_h1 = (uint16_t)(((uint16_t)buffer2[2] << 4) | (buffer2[1] & 0x0f));
  bme680.cp.par_h2 = (uint16_t)(((uint16_t)buffer2[0] << 4) | ((buffer2[1]) >> 4));
  bme680.cp.par_h3 = (int8_t)buffer2[3];
  bme680.cp.par_h4 = (int8_t)buffer2[4];
  bme680.cp.par_h5 = (int8_t)buffer2[5];
  bme680.cp.par_h6 = (uint8_t)buffer2[6];
  bme680.cp.par_h7 = (int8_t)buffer2[7];

  bme680.cp.par_gh1 = (int8_t)buffer2[12];
  bme680.cp.par_gh2 = (int16_t)(bme680_CONCAT_BYTES(buffer2[11], buffer2[10]));
  bme680.cp.par_gh3 = (int8_t)(int8_t)(buffer2[13]);

  bme680.cp.res_heat_val = (int8_t)buffer3[0];
  bme680.cp.res_heat_range = ((buffer3[2] & 0x30) >> 4);
  bme680.cp.range_sw_err = ((int8_t)(buffer3[4] & 0xF0)) / 16;

}

void bme680_reinit(const nrf_drv_twi_t *l_twi)
{

  p_twi = l_twi;
}

int bme680_is_measuring(void)
{
  uint8_t s;

  bme680_read_reg(bme680REG_STATUS, 1, &s);
  if (s & 0b00001000)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * @brief Read new raw values.
 */
void bme680_read_measurements()
{

  uint8_t buffer[15];

  bme680_read_reg(0x1D, 15, buffer);

  bme680.new_data = buffer[0] & 0x80;
  bme680.heatr_stab = buffer[14] & 0x10;

  bme680.adc_p = (uint32_t)(((uint32_t)buffer[2] << 12) | ((uint32_t)buffer[3] << 4)
			    | ((uint32_t)buffer[4] >> 4));
  bme680.adc_t = (uint32_t)(((uint32_t)buffer[5] << 12) | ((uint32_t)buffer[6] << 4)
			   | ((uint32_t)buffer[7] >> 4));
  bme680.adc_h = (uint16_t)(((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9]);
  bme680.adc_g = (uint16_t)((uint32_t)buffer[13] << 2 | (((uint32_t)buffer[14]) >> 6));
  bme680.gas_range = buffer[14] & 0x0F;

}

int32_t bme680_calc_temp(uint32_t adc_temp) {

    int64_t var1, var2, var3;

	var1 = ((int32_t)adc_temp >> 3) - ((int32_t)bme680.cp.par_t1<< 1);
	var2 = (var1 * (int32_t)bme680.cp.par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t)bme680.cp.par_t3 << 4)) >> 14;
	bme680.t_fine = var2 + var3;
	bme680.temperature = ((bme680.t_fine * 5) + 128) >> 8;
	return bme680.temperature;
}
uint32_t bme680_calc_humi(uint32_t adc_humi) {

    int32_t var1, var2_1, var2_2, var2, var3, var4, var5, var6;
	int32_t temp_scaled;

	temp_scaled = (((int32_t)bme680.t_fine * 5) + 128) >> 8;
	var1 = (int32_t)(bme680.adc_h - ((int32_t)((int32_t)bme680.cp.par_h1 * 16))) -
	       (((temp_scaled * (int32_t)bme680.cp.par_h3)
		 / ((int32_t)100)) >> 1);
	var2_1 = (int32_t)bme680.cp.par_h2;
	var2_2 = ((temp_scaled * (int32_t)bme680.cp.par_h4) / (int32_t)100)
		 + (((temp_scaled * ((temp_scaled * (int32_t)bme680.cp.par_h5)
				     / ((int32_t)100))) >> 6) / ((int32_t)100))
		 +  (int32_t)(1 << 14);
	var2 = (var2_1 * var2_2) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t)bme680.cp.par_h6 << 7;
	var4 = ((var4) + ((temp_scaled * (int32_t)bme680.cp.par_h7) /
			  ((int32_t)100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	bme680.humidity = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;
	return bme680.humidity;

}
uint32_t bme680_calc_press(uint32_t adc_press) {

    int32_t var1, var2, var3, var4, calc_press;

	var1 = (((int32_t)bme680.t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
		(int32_t)bme680.cp.par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)bme680.cp.par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)bme680.cp.par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
		 ((int32_t)bme680.cp.par_p3 << 5)) >> 3)
	       + (((int32_t)bme680.cp.par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)bme680.cp.par_p1) >> 15;
	calc_press = 1048576 - adc_press;
	calc_press = (calc_press - (var2 >> 12)) * ((uint32_t)3125);
	var4 = (1 << 31);
	if(calc_press >= var4){
        calc_press = ((calc_press / (uint32_t)var1) << 1);
	}
	else{
        calc_press = ((calc_press << 1) / (uint32_t)var1);
	}


	var1 = ((int32_t)bme680.cp.par_p9 *
		(int32_t)(((calc_press >> 3)
			 * (calc_press >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(calc_press >> 2) * (int32_t)bme680.cp.par_p8) >> 13;
	var3 = ((int32_t)(calc_press >> 8) * (int32_t)(calc_press >> 8)
		* (int32_t)(calc_press >> 8)
		* (int32_t)bme680.cp.par_p10) >> 17;

	bme680.pressure = calc_press
			   + ((var1 + var2 + var3
			       + ((int32_t)bme680.cp.par_p7 << 7)) >> 4);
    return bme680.pressure;

}

uint32_t bme680_calc_gas_res(uint32_t adc_gas, uint8_t gas_range) {
    int64_t var1, var3;
	uint64_t var2;


	static const uint32_t look_up1[16] = { 2147483647, 2147483647, 2147483647,
			       2147483647, 2147483647, 2126008810, 2147483647,
			       2130303777, 2147483647, 2147483647, 2143188679,
			       2136746228, 2147483647, 2126008810, 2147483647,
			       2147483647 };

	static const uint32_t look_up2[16] = { 4096000000, 2048000000, 1024000000,
			       512000000, 255744255, 127110228, 64000000,
			       32258064, 16016016, 8000000, 4000000, 2000000,
			       1000000, 500000, 250000, 125000 };

	var1 = (int64_t)((1340 + (5 * (int64_t)bme680.cp.range_sw_err)) *
		       ((int64_t)look_up1[gas_range])) >> 16;
	var2 = (((int64_t)((int64_t)adc_gas << 15) - (int64_t)(16777216)) + var1);
	var3 = (((int64_t)look_up2[gas_range] * (int64_t)var1) >> 9);
	return (uint32_t)((var3 + ((int64_t)var2 >> 1))
					    / (int64_t)var2);


    //uint32_t msl = bme680.humidity & 0x3FF;
    //uint32_t h = (msh* 1000) + (msl);

}


uint8_t bme680_calc_gas_wait(uint16_t dur) {
    uint8_t factor = 0,durval;

    if (dur >= 0xfc0) {
        durval = 0xff;
    }
    else {
        while (dur > 0x3F) {
            dur = dur / 4;
            factor += 1;
        }

        durval = dur + (factor * 64);
    }

    return durval;

}


uint8_t bme680_calc_res_heat(uint16_t heatr_temp) {

	uint8_t heatr_res;
	int32_t var1, var2, var3, var4, var5;
	int32_t heatr_res_x100;
	int32_t amb_temp = 25;    /* Assume ambient temperature to be 25 deg C */

	if (heatr_temp > 400) { /* Cap temperature */
		heatr_temp = 400;
	}

	var1 = ((amb_temp * bme680.cp.par_gh3) / 1000) * 256;
	var2 = (bme680.cp.par_gh1 + 784) * (((((bme680.cp.par_gh2 + 154009)
					   * heatr_temp * 5) / 100)
					 + 3276800) / 10);
	var3 = var1 + (var2 / 2);
	var4 = (var3 / (bme680.cp.res_heat_range + 4));
	var5 = (131 * bme680.cp.res_heat_val) + 65536;
	heatr_res_x100 = ((var4 / var5) - 250) * 34;
	heatr_res = (heatr_res_x100 + 50) / 100;

	return heatr_res;
}

// For optimisation purpose, oversampling is set once on measure triggering
//any change should be set in local variables not latent in sensor's registers
//that avoid reads before writes
void bme680_measure(void)
{
  //trigger the start of measurments in the sensor
  bme680_write_reg(bme680REG_CTRL_HUM, 0x01);//oversampling hum x1
  bme680_write_reg(bme680REG_CTRL_MEAS, 0x20 | 0x04 | 0x01);//t_x1 | p_x1 | forced
  bme680_write_reg(bme680REG_CONFIG, 0x00);

  bme680_write_reg(bme680REG_CTRL_GAS_1, 0x10);
  bme680_write_reg(bme680REG_RES_HEAT_0, bme680_calc_res_heat(200));
  bme680_write_reg(bme680REG_GAS_WAIT_0, bme680_calc_gas_wait(50));
  bme680_write_reg(bme680REG_CTRL_MEAS, 0x20 | 0x04 | 0x01);//t_x1 | p_x1 | forced


  uint8_t count = 0;
  NRF_LOG_DEBUG("BME done measuring in %u cycles",count);
  //read the data buffer from the sensor
  bme680_read_measurements();
  bme680.temperature = bme680_calc_temp(bme680.adc_t);//temperature must be compensated first
  bme680.pressure = bme680_calc_press(bme680.adc_p);
  bme680.humidity = bme680_calc_humi(bme680.adc_h);
  bme680.gas = bme680_calc_gas_res(bme680.adc_g, bme680.gas_range);


}

/**
 * Returns temperature in DegC, resolution is 0.01 DegC.
 * Output value of “2134” equals 21.34 DegC.
 */
int32_t bme680_get_temperature(void)
{
  return bme680.temperature;
}

/**
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format
 * (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 */
uint32_t bme680_get_pressure(void)
{
  return bme680.pressure;
}

/**
 * Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format
 * (22 integer and 10 fractional bits).
 * Output value of “50532” represents 50532/1024 = 49.356 %RH
 */
uint32_t bme680_get_humidity(void)
{
  return bme680.humidity;
}


uint32_t bme680_get_gas(void)
{
  return bme680.gas;
}


void bme680_deinit(void)
{
        bme680_write_reg(bme680REG_CTRL_MEAS, 0x00 | 0x00 | 0x00);

}

