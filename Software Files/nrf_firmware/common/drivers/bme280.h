#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <stdbool.h>

#include "nrf_drv_twi.h"

struct comp_params {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
};

struct bme280_driver {
	bool sensor_available;
	int32_t adc_h;		///< RAW humidity
	int32_t adc_t;		///< RAW temp
	int32_t adc_p;		///< RAW pressure
	int32_t t_fine;		///< calibrated temp
    struct comp_params cp;	///< calibration data
	int32_t  temperature;	//finalised temp in 0.01° steps
	uint32_t pressure;		//finalised hum in x256 x100 hPa
	uint32_t humidity;		//finalised hum in  x1024
};

extern struct bme280_driver bme280;

enum BME280_MODE {
	BME280_MODE_SLEEP  = 0x00,
	BME280_MODE_FORCED = 0x01,
	BME280_MODE_NORMAL = 0x03
} BME280_Mode_t;

/** States of the module */
typedef enum
{
    BME280_RET_OK = 0,                  /**< Ok */
    BME280_NOT_SUPPORTED = 1,           /**< Feature not supported at the moment */
    BME280_INVALID = 2,                 /**< Returned data may be not valid, because of Power Down Mode or Data not ready */
    BME280_RET_NULL = 4,                /**< NULL Pointer detected */
    BME280_RET_ERROR_SELFTEST = 8,      /**< Selftest  failed */
    BME280_RET_ILLEGAL = 16,            /**< Unallowed configuration, i.e. adjusting configuration while not in sleep.*/
    BME280_RET_ERROR = 32               /**< Not otherwise specified error */
} BME280_Ret;

#define BME280REG_CALIB_00       (0x88)
#define BME280REG_ID             (0xD0)
#define BME280REG_RESET          (0xE0)
#define BME280REG_CALIB_26       (0xE1)
#define BME280REG_CTRL_HUM       (0xF2)
#define BME280REG_STATUS         (0xF3)
#define BME280REG_CTRL_MEAS      (0xF4)
#define BME280REG_CONFIG         (0xF5)
#define BME280REG_PRESS_MSB      (0xF7)
#define BME280REG_PRESS_LSB      (0xF8)
#define BME280REG_PRESS_XLSB     (0xF9)
#define BME280REG_TEMP_MSB       (0xFA)
#define BME280REG_TEMP_LSB       (0xFB)
#define BME280REG_TEMP_XLSB      (0xFC)
#define BME280REG_HUM_MSB        (0xFD)
#define BME280REG_HUM_LSB        (0xFE)

#define BME280_ID_VALUE          (0x60)

#define BME280_OVERSAMPLING_SKIP (0x00)
#define BME280_OVERSAMPLING_1    (0x01)
#define BME280_OVERSAMPLING_2    (0x02)
#define BME280_OVERSAMPLING_4    (0x03)
#define BME280_OVERSAMPLING_8    (0x04)
#define BME280_OVERSAMPLING_16   (0x05)

#define BME280_IIR_MASK          (0x1C)
#define BME280_IIR_OFF           (0x00)
#define BME280_IIR_2             (0x04)
#define BME280_IIR_4             (0x08)
#define BME280_IIR_8             (0x0C)
#define BME280_IIR_16            (0x10)

#define BME280_INTERVAL_MASK     (0xE0)

#define BME280_BURST_READ_LENGTH (8) // 8 bytes only for I²C
#define BME280_MAX_READ_LENGTH BME280_BURST_READ_LENGTH

enum BME280_INTERVAL {
	BME280_STANDBY_0_5_MS  = 0x0,
	BME280_STANDBY_62_5_MS = 0x20,
	BME280_STANDBY_125_MS  = 0x40,
	BME280_STANDBY_500_MS  = 0x80,
	BME280_STANDBY_1000_MS = 0xA0
};

/** Structure containing sensor data from all 3 sensors */
typedef struct {
  int32_t  temperature;
  uint32_t humidity;
  uint32_t pressure;
}bme280_data_t;

/**
 *  Initialises BME280 in sleep mode, all sensors enabled
 */
BME280_Ret bme280_init(const nrf_drv_twi_t *l_twi);

BME280_Ret bme280_reinit(const nrf_drv_twi_t *l_twi);

void bme280_deinit();

/**
 * triggers measurements on the device, wait for them to complete
 * then reads the data from the sensor
 * All together, temp, hum, press with x1 sample
 */
void bme280_measure();

/**
 * dumps config registers 0xF2, 0xF3, 0xF4, 0xF5
 * ctrl_hum, status, ctrl_meas, config
 */
void bme280_dump();

/**
 *  Return true if measurement is in progress
 */

int  bme280_is_measuring(void);

/**
 *  Read measurements from BME280 to nRF52.
 *  You have to call this manually, in normal mode you get latest stored values.
 *  Forced mode can be used as:
 *  bme_set_mode(BME280_MODE_FORCED)
 *  while(bme280_is_measuroing());
 *  bme280_read_measurements();
 */
BME280_Ret bme280_read_measurements();

/**
 * Returns temperature in DegC, resolution is 0.01 DegC.
 * Output value of “2134” equals 21.34 DegC.
 */
int32_t  bme280_get_temperature(void);

 /**
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format
 * (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 */
uint32_t bme280_get_pressure(void);

/**
 * Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format
 * (22 integer and 10 fractional bits).
 * Output value of “50532” represents 50532/1024 = 49.356 %RH
 */
uint32_t   bme280_get_humidity(void);

#endif
