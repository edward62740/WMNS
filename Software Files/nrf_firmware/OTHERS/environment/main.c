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
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_pwr_mgmt.h"

//log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//drivers
#include "bme280.h"
#include "opt3001.h"
#include "clocks.h"
#include "battery.h"
#include "twi.h"
#include "mesh.h"


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

void app_mesh_handler(message_t* msg)
{
    NRF_LOG_INFO("app_mesh_handler()");
}

void GPIOTE_IRQHandler(void)
{
    *(volatile uint32_t *)0x40000EE4 = (*(volatile uint32_t *)0x10000258 & 0x0000004F);
    clocks_restart();
    twi_restart();



    NVIC_DisableIRQ(GPIOTE_IRQn); // disable GPIOTE to prevent interrupt firing after sensor has been read

    if(NRF_GPIOTE->EVENTS_PORT)
    {
        NRF_GPIOTE->EVENTS_PORT = 0;

        int32_t temp    = bme280_get_temperature();
        uint32_t hum    = bme280_get_humidity();
        uint32_t press  = bme280_get_pressure();
        uint16_t raw_light = opt3001_read();
        uint32_t light = opt3001_conv(raw_light);

        mesh_tx_env(temp,hum,press,light);

        opt3001_deinit();
        bme280_deinit();
    }
    twi_stop();
    clocks_stop();



}

void app_rtc_handler()
{
    *(volatile uint32_t *)0x40000EE4 = (*(volatile uint32_t *)0x10000258 & 0x0000004F);
    clocks_restart();
    twi_restart();
    nrf_gpio_cfg_output(4);
    nrf_gpio_pin_set(4);

    /* Declare static counts */
    static uint32_t cycle_count = 0;
    static const uint32_t period_env  = 1;
    static const uint32_t offset_env  = 0;
    static const uint32_t period_bat    = 20;
    static const uint32_t offset_bat    = 1;
    static const uint32_t period_alive  = 20;
    static const uint32_t offset_alive  = 5;

    /* Measure battery voltage and transmit message if bat period */
    if( ((cycle_count+offset_bat) % period_bat)==0)
    {
        uint16_t v_bat_mili = get_battery();
        mesh_tx_battery(v_bat_mili);
        #if(NRF_LOG_LEVEL <= NRF_LOG_LEVEL_INFO)
        float v_bat_mili_f  = v_bat_mili;
        NRF_LOG_INFO("V Bat = "NRF_LOG_FLOAT_MARKER" Volts",NRF_LOG_FLOAT(v_bat_mili_f/1000));
        #endif
        mesh_wait_tx();
    }

    /* Transmit alive packet if alive period */
    else if( ((cycle_count+offset_alive) % period_alive)==0)
    {
        mesh_tx_alive();
        mesh_wait_tx();
    }

    /* Initialize GPIOTE and start measurement if env period */
    else if( ((cycle_count+offset_env) % period_env)==0)
    {
        opt3001_init(&m_twi);
        bme280_init(&m_twi);
        bme280_measure();

        NRF_GPIOTE->EVENTS_PORT = 0;
        NVIC_ClearPendingIRQ(GPIOTE_IRQn);
        nrf_gpio_cfg_sense_input(16, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
        NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
        NVIC_SetPriority(GPIOTE_IRQn, 7);
        NVIC_EnableIRQ(GPIOTE_IRQn);
    }

    nrf_gpio_pin_clear(4);
    nrf_gpio_cfg_default(4);

    cycle_count++;
    twi_stop();
    clocks_stop();


}



int main(void)
{
    *(volatile uint32_t *)0x40000EE4 = (*(volatile uint32_t *)0x10000258 & 0x0000004F);

    uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    for(int i = 0; i < 32; i++) { nrf_gpio_cfg_default(i); }
    //NRF_POWER->DCDCEN = 1;
    clocks_start();
    battery_init();
    err_code = mesh_init(app_mesh_handler,NULL);//in case of rf rx, cmd handler not required
    APP_ERROR_CHECK(err_code);
    twi_init(&m_twi);
    rtc_config(app_rtc_handler);
    mesh_ttl_set(3);


    
    mesh_tx_reset();
    mesh_wait_tx();
    twi_stop();
    clocks_stop();//release the hf clock
    //NRF_TIMER0-> TASKS_SHUTDOWN = 1;
    while(true)
    {
        nrf_pwr_mgmt_run();
    }
}

