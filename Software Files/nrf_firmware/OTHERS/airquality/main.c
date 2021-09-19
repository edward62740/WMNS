#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_util.h"


#include "nrf_pwr_mgmt.h"

//log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


//drivers

#include "opt3001.h"
//apps
#include "clocks.h"
#include "battery.h"
#include "twi.h"
#include "nrf_drv_twi.h"
#include "mesh.h"
#include "bsec_integration.h"
#include "bme680.h"
#include "bsec_interface.h"

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static const nrf_drv_twi_t *p_twi =  NULL;
static const uint8_t dev_addr = 0x76;

void app_mesh_handler(message_t* msg)
{
    NRF_LOG_INFO("app_mesh_handler()");
}

int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{

  ret_code_t err_code;
  err_code = nrf_drv_twi_tx(p_twi, dev_addr, &reg_addr, 1,true);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_twi_rx(p_twi, dev_addr, reg_data_ptr, data_len);
  APP_ERROR_CHECK(err_code);

  return 0;

}


int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{


  ret_code_t err_code;
  uint8_t data[data_len+1];
  data[0] = reg_addr;
  for (int index = 1; index < data_len + 1; index++) {
    data[index] = *reg_data_ptr;
  }

  err_code = nrf_drv_twi_tx(p_twi, dev_addr, data, data_len,false);
  APP_ERROR_CHECK(err_code);

  return 0;
}

void sleep(uint32_t t_ms)
{
    nrf_delay_ms(3000);
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    int64_t system_current_time = 0;
    // ...
    // Please insert system specific function to retrieve a timestamp (in microseconds)
    // ...
    return system_current_time;
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    mesh_tx_iaq((int32_t)temperature, (uint32_t)humidity, (uint32_t)pressure, (uint32_t)co2_equivalent, (uint32_t)breath_voc_equivalent);
    mesh_wait_tx();
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...
    return 0;
}

void app_rtc_handler()
{
nrf_gpio_pin_set(4);
    mesh_tx_alive();
    mesh_wait_tx();
    nrf_delay_ms(500);
    nrf_gpio_pin_clear(4);
}
int main(void)
{
    uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    for(int i = 0; i < 32; i++) { nrf_gpio_cfg_default(i); }
    //NRF_POWER->DCDCEN = 1;
    clocks_start();

    APP_ERROR_CHECK(err_code);
    twi_init(&m_twi);

    nrf_gpio_cfg_output(4);
    nrf_gpio_pin_set(4);
    return_values_init ret;
NRF_LOG_DEBUG("ERROR CHECK %X\r\n", 1);
    /* Call to the function which initializes the BSEC library
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, sleep, state_load, config_load);
    NRF_LOG_ERROR("ERROR RETURN INIT %X\r\n", ret.bme680_status);
 NRF_LOG_ERROR("ERROR RETURN INIT %X\r\n", ret.bsec_status);

    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */

 
}
/*lint -restore */
