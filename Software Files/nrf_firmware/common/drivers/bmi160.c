#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "bmi160.h"
#include "math.h"

static const uint8_t address = 0x68;
static const nrf_drv_twi_t *p_twi = NULL;

uint8_t bmi160_read_reg(uint8_t reg)
{
   uint8_t res;
   ret_code_t err_code;
   err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
   APP_ERROR_CHECK(err_code);
   err_code = nrf_drv_twi_rx(p_twi, address, &res, 1);
   APP_ERROR_CHECK(err_code);

   return res;
}

void bmi160_write_reg(uint8_t reg, uint8_t data)
{
   uint8_t buffer[2];
   buffer[0] = reg;
   buffer[1] = data;

   ret_code_t err_code;
   err_code = nrf_drv_twi_tx(p_twi, address, buffer, 2, true);
   APP_ERROR_CHECK(err_code);
}

uint8_t bmi160_read_reg_bits(uint8_t reg, unsigned pos, unsigned len)
{
   uint8_t b = bmi160_read_reg(reg);
   uint8_t mask = (1 << len) - 1;
   b >>= pos;
   b &= mask;

   return b;
}

void bmi160_write_reg_bits(uint8_t reg, uint8_t data, unsigned pos, unsigned len)
{
   uint8_t b = bmi160_read_reg(reg);
   uint8_t mask = ((1 << len) - 1) << pos;
   data <<= pos; // shift data into correct position
   data &= mask; // zero all non-important bits in data
   b &= ~(mask); // zero all important bits in existing byte
   b |= data; // combine data with existing byte
   bmi160_write_reg(reg, b);
}

void bmi160_init(const nrf_drv_twi_t *l_twi)
{
   p_twi = l_twi;
   //bmi160_write_reg(0x7E, 0xB1);
   //bmi160_write_reg(BMI160_RA_CMD, BMI160_CMD_GYR_MODE_NORMAL);

  

}

bool bmi160_check_pmu()
{
   bool ret;
   ret = bmi160_read_reg_bits(0x03, 0, 6);
   if (ret == 0b000000) {return true;}
   else {return false;}
}

void bmi160_set_sigmotion()
{  
   bmi160_write_reg(0x40, 162);
   bmi160_write_reg_bits(0x41, 3, 0, 4);

   bmi160_write_reg(0x7E, 0x12);
   //bmi160_write_reg_bits(0x40, 1, 7, 1);
   bmi160_write_reg(0x60, 8); // threshold



   /*bmi160_write_reg(0x5F, 1); // slope

   bmi160_write_reg(0x60, 2); // threshold

   bmi160_write_reg_bits(0x62, 1, 1, 1); // select sig motion

   bmi160_write_reg_bits(0x62, 2, 4, 2); // proof time

   bmi160_write_reg_bits(0x62, 1, 2, 2); // skip time
*/

   
   /*bmi160_write_reg(0x51, 0);
   bmi160_write_reg(0x52, 0);*/
bmi160_write_reg_bits(0x50, 7, 0, 3); // enable interrupt engine for sig motion




}

void bmi160_disableGyro()
{
   //bmi160_write_reg(0x
}

void bmi160_set_interrupt()
{

   bmi160_write_reg(0x55, 0xFF); // map all to INT1
   bmi160_write_reg(0x56, 0xF0);
   bmi160_write_reg(0x57, 0x00);
   bmi160_write_reg_bits(0x59, 0, 7, 1); // int source
   bmi160_write_reg(0x53, 8); // Enable INT1, active low, push-pull
   bmi160_write_reg_bits(0x54, 9, 0, 4); // latch 640ms
}

void bmi160_low_power()
{

   bmi160_write_reg(0x40, 162);
   bmi160_write_reg_bits(0x41, 3, 2, 1);
   bmi160_write_reg(0x7E, 0xA0);

   
}
uint16_t stat()
{
   uint8_t buffer[2];
   buffer[0] = bmi160_read_reg(0x1C);
   //buffer[1] = bmi160_read_reg(0x1A);
   return (((int16_t)buffer[1]) << 8) | buffer[0];
}

