/*
* This file is part of VL53L1 Platform
*
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "math.h"
#include "nrf_delay.h"
static const nrf_drv_twi_t *p_twi = NULL;

void VL53L1(const nrf_drv_twi_t *l_twi)
{
        p_twi = l_twi;
}

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    ret_code_t ret;

	/* device supports only limited number of bytes written in sequence */
	if (count > (256))
	{
		return NRF_ERROR_INVALID_LENGTH;
	}

	/* All written data has to be in the same page */
	if ((dev / (256)) != ((index + count - 1) / (256)))
	{
		return NRF_ERROR_INVALID_ADDR;
	}

	do{
		uint8_t buffer[2 + 256]; /* index + data */
                buffer[0] = (index >> 8) & 0xFF;
                buffer[1] = index & 0xFF;
		memcpy(buffer + 2, pdata, count);
		ret = nrf_drv_twi_tx(p_twi, dev >> 1, buffer, count + 2, false);
	} while (0);
	return ret;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
        ret_code_t ret;

    if (count > (256))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    do
    {
       uint8_t buffer[2];
       buffer[0] = (index >> 8) & 0xFF;
       buffer[1] = index & 0xFF;
       ret = nrf_drv_twi_tx(p_twi, dev >> 1, buffer, 2, true);
       if (NRF_SUCCESS != ret)
       {
    	   return 1;
       }
       ret = nrf_drv_twi_rx(p_twi, dev >> 1, pdata, count);
    }while (0);
    return ret;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{

    ret_code_t ret;

    do
    {
        uint8_t buffer[3];
        buffer[0] = (index >> 8) & 0xFF;
        buffer[1] = index & 0xFF;
        buffer[2] = data & 0xFF;
        ret = nrf_drv_twi_tx(p_twi, dev >> 1, buffer, 3, false);
    }
    while (0);
    return ret;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    ret_code_t ret;
    do
    {
        uint8_t buffer[4];
        buffer[0] = (index >> 8) & 0xFF;
        buffer[1] = index & 0xFF;
        buffer[2] = (data >> 8) & 0xFF;
        buffer[3] = data & 0xFF;
        ret = nrf_drv_twi_tx(p_twi, dev >> 1, buffer, 4, false);
    }
    while (0);

    return ret;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{

    ret_code_t ret;
    do
    {
        uint8_t buffer[6];
        buffer[0] = (index >> 8) & 0xFF;
        buffer[1] = index & 0xFF;
        buffer[2] = (data >> 24) & 0xFF;
        buffer[3] = (data >> 16) & 0xFF;
        buffer[4] = (data >> 8) & 0xFF;
        buffer[5] = data & 0xFF;
        ret = nrf_drv_twi_tx(p_twi, dev >> 1, buffer, 6, false);
    }
    while (0);

    return ret;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
    ret_code_t ret;
    uint8_t buffer[2];
    buffer[0] = (index >> 8) & 0xFF;
    buffer[1] = index & 0xFF;
    do
    {
        ret = nrf_drv_twi_tx(p_twi, dev >> 1, buffer, 2, true);
        if (NRF_SUCCESS != ret)
        {
            return 1;
        }
        ret = nrf_drv_twi_rx(p_twi, dev >> 1, data, 1);
    }
    while (0);
    return ret;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
    ret_code_t ret;
    uint8_t buffer[2];
    buffer[0] = (index >> 8) & 0xFF;
    buffer[1] = index & 0xFF;
    do
    {
        uint8_t rxbuffer[2];
        ret = nrf_drv_twi_tx(p_twi, dev >> 1, buffer, 2, true);
        if (NRF_SUCCESS != ret)
        {
            return 1;
        }
        ret = nrf_drv_twi_rx(p_twi, dev >> 1, rxbuffer, 2);
        *data = (rxbuffer[0] << 8) + rxbuffer[1];
    }
    while (0);
    return ret;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
   ret_code_t ret;
    uint8_t buffer[2];
    buffer[0] = (index >> 8) & 0xFF;
    buffer[1] = index & 0xFF;
    do
    {
        uint8_t rxbuffer[4];
        ret = nrf_drv_twi_tx(p_twi, dev >> 1, buffer, 2, true);
        if (NRF_SUCCESS != ret)
        {
            return 1;
        }
        ret = nrf_drv_twi_rx(p_twi, dev >> 1, rxbuffer, 4);
        *data = (rxbuffer[0] << 24) + (rxbuffer[1] << 16) + (rxbuffer[2] << 8) + rxbuffer[3];
    }
    while (0);
    return ret;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    nrf_delay_ms((uint32_t)wait_ms);
    return 0; // to be implemented
}
