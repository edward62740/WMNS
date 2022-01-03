#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "bma400.h"
#include "math.h"

static const uint8_t address = 0x14;
static const nrf_drv_twi_t *p_twi = NULL;

uint8_t bma400_read_reg(uint8_t reg)
{
   uint8_t res;
   ret_code_t err_code;
   err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
   APP_ERROR_CHECK(err_code);
   err_code = nrf_drv_twi_rx(p_twi, address, &res, 1);
   APP_ERROR_CHECK(err_code);

   return res;
}

uint8_t bma400_read_burst(uint8_t start, uint8_t length, uint8_t* buffer)
{
  ret_code_t err_code;
  err_code = nrf_drv_twi_tx(p_twi, address, &start, 1,true);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_twi_rx(p_twi, address, buffer, length);
  APP_ERROR_CHECK(err_code);

  return 0;
}

void bma400_write_reg(uint8_t reg, uint8_t data)
{
   uint8_t buffer[2];
   buffer[0] = reg;
   buffer[1] = data;

   ret_code_t err_code;
   err_code = nrf_drv_twi_tx(p_twi, address, buffer, 2, true);
   APP_ERROR_CHECK(err_code);
}

uint8_t bma400_read_reg_bits(uint8_t reg, unsigned pos, unsigned len)
{
   uint8_t b = bma400_read_reg(reg);
   uint8_t mask = (1 << len) - 1;
   b >>= pos;
   b &= mask;

   return b;
}

void bma400_write_reg_bits(uint8_t reg, uint8_t data, unsigned pos, unsigned len)
{
   uint8_t b = bma400_read_reg(reg);
   uint8_t mask = ((1 << len) - 1) << pos;
   data <<= pos; // shift data into correct position
   data &= mask; // zero all non-important bits in data
   b &= ~(mask); // zero all important bits in existing byte
   b |= data; // combine data with existing byte
   bma400_write_reg(reg, b);
}

void bma400_init(const nrf_drv_twi_t *l_twi)
{
   p_twi = l_twi;
}

bool bma400_verify(void)
{
    if(bma400_read_reg(0x00) == 0x90){
        return true;
    }
    else return false;
}

void bma400_reset(void)
{

    bma400_write_reg(0x7E, 0xB6);
}

void bma400_set_power_mode(uint8_t power)
{

    uint8_t data = bma400_read_reg(0x19);
    data = (data & 0xFC) | power;
    bma400_write_reg(0x19, data);
}

void bma400_set_range(uint8_t range)
{


    if (range == 0x00) {
        accRange = 2000;
    } else if (range == 0x01) {
        accRange = 4000;
    } else if (range == 0x02) {
        accRange = 8000;
    } else if (range == 0x03) {
        accRange = 16000;
    }

    uint8_t data = bma400_read_reg(0x1A);
    data = (data & 0x3f) | (range << 6);
    bma400_write_reg(0x1A, data);

}

void bma400_set_odr(uint8_t odr)
{
    uint8_t data = bma400_read_reg(0x1A);
    data = (data & 0xF0) | odr;
    bma400_write_reg(0x1A, data);
}

void bma400_get_accel(float *x, float *y, float *z) {

    uint8_t buf[6] = {0};
    uint16_t ax = 0, ay = 0, az = 0;
    float value = 0;

    bma400_read_burst(0x04, 6, buf);

    ax = buf[0] | (buf[1] << 8);
    ay = buf[2] | (buf[3] << 8);
    az = buf[4] | (buf[5] << 8);

    if (ax > 2047) {
        ax = ax - 4096;
    }
    if (ay > 2047) {
        ay = ay - 4096;
    }
    if (az > 2047) {
        az = az - 4096;
    }

    value = (int16_t)ax;
    *x = accRange * value / 2048;

    value = (int16_t)ay;
    *y = accRange * value / 2048;

    value = (int16_t)az;
    *z = accRange * value / 2048;
}

int16_t bma400_get_temp(void) {

    int8_t data = 0;
    int16_t temp = 0;

    data = (int8_t)bma400_read_reg(0x11);
    temp = data / 2 + 23;
    return temp;
}

uint32_t bma400_get_time(void) {

    uint32_t time;
    uint8_t buf[3] = {0};

    bma400_read_burst(0x0A, 3, buf);
    time = buf[0] >> 3;
    time |= buf[1] << 5;
    time |= buf[2] << 13;

    time = (uint32_t)time*100;
    return time;
}

void bma400_set_interrupt_on_activity(void) {


    bma400_write_reg(0x21, 0x04);
    bma400_write_reg(0x20, 0x00);
    bma400_write_reg(0x24, 0x02);

    //bma400_write_reg_bits(0x2B, 0x01, 1, 1); // trigger switching from normal to LP mode
    bma400_write_reg(0x3F, 0xFA); // enable 3 axis, select 100Hz data source, hyst 48mg
    bma400_write_reg(0x40, 0x02); // OR, activity interrupt
    bma400_write_reg(0x41, 0x01); // 8mg/LSB

    bma400_write_reg(0x42, 0x00);
    bma400_write_reg(0x43, 0x0F); // 15 ODR ticks
    bma400_write_reg(0x1F, 0x04); // map gen1 to int1

}

void bma400_set_step_count(void) {

    bma400_write_reg(0x20, 0x01); // enable step detected interrupt

}

uint32_t bma400_get_step_count(void) {

    uint32_t step;
    uint8_t buf[3] = {0};

    bma400_read_burst(0x15, 3, buf);
    step = buf[0];
    step |= buf[1] << 8;
    step |= buf[2] << 16;

    step = (uint32_t)step;
    return step;

}
