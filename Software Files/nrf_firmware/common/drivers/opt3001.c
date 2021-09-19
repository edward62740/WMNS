#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "opt3001.h"
#include "math.h"

#define REG_RESULT                      0x00
#define REG_CONFIGURATION               0x01
#define REG_LOWLIMIT                    0x02


static const uint8_t address = 0x45;
static const nrf_drv_twi_t *p_twi = NULL;


uint16_t opt3001_read_reg(uint8_t reg)
{

   uint8_t res[2];

   ret_code_t err_code;
   err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
   APP_ERROR_CHECK(err_code);
   err_code = nrf_drv_twi_rx(p_twi, address, res, 2);
   APP_ERROR_CHECK(err_code);

   return ((uint16_t) res[0] << 8) | res[1];
}

void opt3001_write_reg(uint8_t reg, uint8_t dataL, uint8_t dataH)
{

   uint8_t data[3];
   data[0] = reg;
   data[1] = dataL;
   data[2] = dataH;

   ret_code_t err_code;
   err_code = nrf_drv_twi_tx(p_twi, address, data, 3, false);
   APP_ERROR_CHECK(err_code);


}
void opt3001_init(const nrf_drv_twi_t *l_twi)
{
    p_twi = l_twi;
    opt3001_write_reg(REG_CONFIGURATION, 0b11000110, 0b00000000); //0b1100001000010000
    opt3001_write_reg(REG_LOWLIMIT, 0b11000000, 0b00000000);


}

void opt3001_deinit(void)
{

    opt3001_write_reg(REG_CONFIGURATION, 0b11000000, 0x10); //0b1100001000010000

}

uint16_t opt3001_read(void)
{
    uint8_t count = 0;
    const uint8_t max_wait = 55;
    int is_measuring;
    do
    {
      is_measuring = opt3001_is_measuring();
      count++;
    }while( (is_measuring == 1) && (count < max_wait) );
    if(count == max_wait)
    {
       return opt3001_read_reg(REG_CONFIGURATION);
    }
    else
    {
       uint16_t raw = opt3001_read_reg(REG_RESULT);
       return raw;
    }

}

int opt3001_is_measuring(void)
{
    uint16_t s;
    s = opt3001_read_reg(REG_CONFIGURATION);
    if (s & (1 << 7))
    {
      return 0;
    }
    else
    {
      return 1;
    }
}

float opt3001_conv(uint16_t raw)
{
    uint16_t e, m;

    m = raw & 0x0FFF;
    e = (raw & 0xF000) >> 12;

    return m * (10 * exp2(e));
}

