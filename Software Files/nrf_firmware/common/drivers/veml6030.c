#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "veml6030.h"
#include "math.h"

static const uint8_t address = 0x48;
static const nrf_drv_twi_t *p_twi = NULL;

const float IT800[4] = { .0036, .0072, .0288, .0576 };
const float IT400[4] = { .0072, .0144, .0576, .1152 };
const float IT200[4] = { .0144, .0288, .1152, .2304 };
const float IT100[4] = { .0288, .0576, .2304, .4608 };
const float IT50[4] = { .0576, .1152, .4608, .9216 };
const float IT25[4] = { .1152, .2304, .9216, 1.8432 };


uint8_t veml6030_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t res[3];
    res[0] = reg;
    res[1] = value ;
    res[2] = value >> 8 ;

    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(p_twi, address, res, 3, false);
    APP_ERROR_CHECK(err_code);
    return 0;
}

uint16_t veml6030_read_reg(uint8_t reg)
{
    uint8_t res[2];

    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(p_twi, address, &reg, 1, true);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_twi_rx(p_twi, address, res, 2);
    APP_ERROR_CHECK(err_code);

    return (uint16_t) ((res[1] << 8) | res[0]);
}

uint32_t veml6030_init(const nrf_drv_twi_t *l_twi)
{
    p_twi = l_twi;

    // Reset VEML configuration (in order to check device)
    return veml6030_write_reg(REG_ALS_CONF, 0);
}

uint32_t veml6030_power_on()
{
    // Get current config and clear shutdown bit
    uint16_t config = veml6030_read_reg(REG_ALS_CONF);
    config &= ~REG_ALS_CONF_SHUTDOWN;

    return veml6030_write_reg(REG_ALS_CONF, config);
}

uint32_t veml6030_shutdown()
{
    // Get current config and set shutdown bit
    uint16_t config = veml6030_read_reg(REG_ALS_CONF);
    config |= REG_ALS_CONF_SHUTDOWN;

    return veml6030_write_reg(REG_ALS_CONF, config);
}

uint32_t veml6030_set_als_integration_time(uint16_t it)
{
    uint16_t config = veml6030_read_reg(REG_ALS_CONF);
    config &= ~REG_ALS_CONF_IT_CLEAR;
    config |= it;
    return veml6030_write_reg(REG_ALS_CONF, config);
}

uint16_t veml6030_get_als_integration_time()
{
    uint16_t config = veml6030_read_reg(REG_ALS_CONF);
    return (config & REG_ALS_CONF_IT_CLEAR) >> 6;
}

uint32_t veml6030_set_als_gain(uint16_t gain)
{
    uint16_t config = veml6030_read_reg(REG_ALS_CONF);
    // Clear all gain bits
    config &= ~REG_ALS_CONF_GAIN_1_4;
    config |= gain;
    return veml6030_write_reg(REG_ALS_CONF, config);
}

uint16_t veml6030_get_als_gain()
{
    uint16_t config = veml6030_read_reg(REG_ALS_CONF);
    return (config & REG_ALS_CONF_GAIN_1_4) >> 11;
}



uint32_t veml6030_set_psm(uint16_t psm)
{
    uint16_t config = veml6030_read_reg(REG_POWER_SAVING);
    // Clear all psm bits
    config &= ~REG_POWER_SAVING_PSM_4;
    config |= psm;
    return veml6030_write_reg(REG_POWER_SAVING, config);
}

uint16_t veml6030_get_psm()
{
    uint16_t config = veml6030_read_reg(REG_POWER_SAVING);
    return (config & REG_POWER_SAVING_PSM_4) >> 4;
}

uint32_t veml6030_int_en(uint16_t en)
{
    uint16_t data = veml6030_read_reg(REG_ALS_CONF);
    //data |= ((en & 0x1) << 1);
    data |= 0x2;
    veml6030_write_reg(REG_ALS_WL, 0xFFFE);
    veml6030_write_reg(REG_ALS_WH, 0xFFFF);
    return veml6030_write_reg(REG_ALS_CONF, data);
}

uint16_t veml6030_int_clear()
{

    return (uint16_t)veml6030_read_reg(REG_ALS_INT);
}

bool veml6030_read_res(double *resals, double *resw)
{

    double als = 0, w = 0;
    uint16_t gain = 0, int_time = 0, rals = 0, rw = 0;
    double f_als = 0, cp_als = 0, f_w = 0, cp_w = 0;
    double k = 0;
    int pos = 0;
    rals = veml6030_read_reg(REG_ALS);
    rw = veml6030_read_reg(REG_WHITE);
    als = (double)rals;
    w = (double)rw;
    gain = veml6030_get_als_gain();
    int_time = veml6030_get_als_integration_time();
    if (gain == 0x01)
    {
        pos = 0;
    }
    else if (gain == 0x00)
    {
        pos = 1;
    }
    else if (gain == 0x03)
    {
        pos = 2;
    }
    else if (gain == 0x02)
    {
        pos = 3;
    }

    if (pos != 0 && pos < 5)
    {
        if (int_time == 0x03)
        {
            k = IT800[pos];
        }
        else if (int_time == 0x02)
        {
            k = IT400[pos];
        }
        else if (int_time == 0x01)
        {
            k = IT200[pos];
        }
        else if (int_time == 0x00)
        {
            k = IT100[pos];
        }
        else if (int_time == 0x08)
        {
            k = IT50[pos];
        }
        else if (int_time == 0x0C)
        {
            k = IT25[pos];
        }

        f_als = k * als;
        f_w = k * w;

        if (f_als > 1000 )
        {
            cp_als = (.00000000000060135 * (pow(f_als, 4)))
                     - (.0000000093924 * (pow(f_als, 3)))
                     + (.000081488 * (pow(f_als, 2))) + (1.0023 * f_als);
            *resals = cp_als;
        }
        else
        {
            *resals = f_als;
        }


        if (f_w > 1000)
        {
            cp_w = (.00000000000060135 * (pow(f_w, 4)))
                   - (.0000000093924 * (pow(f_w, 3)))
                   + (.000081488 * (pow(f_w, 2))) + (1.0023 * f_w);
            *resw = cp_w;
        }
        else
        {
            *resw = f_w;
        }

        return true;
    }
    else
        return false;
}
