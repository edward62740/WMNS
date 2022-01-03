#ifndef __BMA400_h__
#define __BMA400_h__
#define __BMA400_h__

#include <stdint.h>

#include "nrf_drv_twi.h"


enum power_type { // power mode
    SLEEP = 0x00, // Stop conversion, the lowest power mode
    LOW_POWER = 0x01, //
    NORMAL = 0x02, //
}power_type_t;


enum range_type { // measurement rage
    RANGE_2G = 0x00, //
    RANGE_4G = 0x01, //
    RANGE_8G = 0x02, //
    RANGE_16G = 0x03, //
}range_type_t;

enum odr_type { // output data rate
    ODR_12 = 0x00, //
    ODR_25 = 0x06, //
    ODR_50 = 0x07, //
    ODR_100 = 0x08, //
    ODR_200 = 0x09, //
    ODR_400 = 0x0A, //
    ODR_800 = 0x0B, //
}odr_type_t;

uint8_t bma400_read_reg(uint8_t reg);
uint8_t bma400_read_burst(uint8_t start, uint8_t length, uint8_t* buffer);
void bma400_write_reg(uint8_t reg, uint8_t data);
uint8_t bma400_read_reg_bits(uint8_t reg, unsigned pos, unsigned len);
void bma400_write_reg_bits(uint8_t reg, uint8_t data, unsigned pos, unsigned len);
void bma400_init(const nrf_drv_twi_t *l_twi);
bool bma400_verify(void);
void bma400_reset(void);
void bma400_set_power_mode(uint8_t power);
void bma400_set_range(uint8_t range);
void bma400_set_odr(uint8_t odr);
void bma400_get_accel(float *x, float *y, float *z);
int16_t bma400_get_temp(void);
uint32_t bma400_get_time(void);
void bma400_set_interrupt_on_activity(void);
void bma400_set_step_count(void);
uint32_t bma400_get_step_count(void);

float accRange;
#endif // __BMA400_h__