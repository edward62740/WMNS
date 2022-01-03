#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_power.h"
#include "boards.h"
#include "app_util.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_pwr_mgmt.h"
#include "uicr_user_defines.h"
//log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//drivers
#include "clocks.h"
#include "battery.h"
#include "twi.h"
#include "mesh.h"
#include "si7021.h"

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1); /**< Declaring an instance of nrf_drv_rtc for RTC0. */
static int32_t temp = 0;
static  uint32_t hum = 0;
void app_mesh_handler(message_t *msg)
{

}

void rtc_handler1(nrf_drv_rtc_int_type_t int_type)
{
    clocks_restart();
    twi_restart();

    if (int_type == NRF_DRV_RTC_INT_COMPARE1)
    {


        nrf_drv_rtc_counter_clear(&rtc1);
           nrf_drv_rtc_disable(&rtc1);

            nrf_gpio_cfg_output(28);
    nrf_gpio_pin_set(28);
            while(!si7021_fetch_humidity_no_hold_master(&hum)) {}
            si7021_fetch_humidity_no_hold_master(&hum);
            si7021_start_temperature_no_hold_master();
   
            while(!si7021_fetch_temperature_no_hold_master(&temp)) {}
            si7021_fetch_temperature_no_hold_master(&temp);
            mesh_tx_lts(temp, hum);
            mesh_wait_tx();
nrf_gpio_pin_clear(28);
             nrf_gpio_cfg_default(28);
            twi_stop();
            clocks_stop();
        }
    

}


void app_rtc_handler()
{
    *(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
                                        & 0x0000004F);
    clocks_restart();
    twi_restart();


    /* Declare static counts */
    static uint32_t cycle_count = 0;
    static const uint32_t period_env = 1;
    static const uint32_t offset_env = 0;
    static const uint32_t period_bat = 360;
    static const uint32_t offset_bat = 1;
    static const uint32_t period_alive = 60;
    static const uint32_t offset_alive = 5;

    /* Measure battery voltage and transmit message if bat period */
    if (((cycle_count + offset_bat) % period_bat) == 0)
    {
        nrf_gpio_cfg_output(28);
    nrf_gpio_pin_set(28);
        uint16_t v_bat_mili = get_battery();
        mesh_tx_battery(v_bat_mili);
        mesh_wait_tx();
        nrf_gpio_pin_clear(28);
    nrf_gpio_cfg_default(28);
    }

    /* Transmit alive packet if alive period */
    else if (((cycle_count + offset_alive) % period_alive) == 0)
    {
        nrf_gpio_cfg_output(28);
    nrf_gpio_pin_set(28);
        mesh_tx_alive();
        mesh_wait_tx();
        nrf_gpio_pin_clear(28);
    nrf_gpio_cfg_default(28);
    }

    /* Initialize GPIOTE and start measurement if env period */
    else if (((cycle_count + offset_env) % period_env) == 0)
    {

        si7021_start_humidity_no_hold_master();


        nrf_drv_clock_lfclk_request(NULL);
        while(!nrf_drv_clock_lfclk_is_running());

        nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
        config.prescaler = 31; // 2^11 - 1

        nrf_drv_rtc_init(&rtc1, &config, rtc_handler1);
        nrf_drv_rtc_tick_enable(&rtc1,false);
        nrf_drv_rtc_cc_set(&rtc1,1,23,true);
        nrf_drv_rtc_enable(&rtc1);
        //si7021_reset();
    }



    cycle_count++;
    twi_stop();
    clocks_stop();

}

int main(void)
{
    *(volatile uint32_t*) 0x40000EE4 = (*(volatile uint32_t*) 0x10000258
                                        & 0x0000004F);


    uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    for (int i = 0; i < 32; i++)
    {
        nrf_gpio_cfg_default(i);
    }
    nrf_gpio_cfg_output(28);
    nrf_gpio_pin_set(28);
    nrf_gpio_cfg_output(29);
    nrf_gpio_pin_set(29);
    NRF_POWER->DCDCEN = 1;
    clocks_start();
    battery_init();
    err_code = mesh_init(app_mesh_handler, NULL); //in case of rf rx, cmd handler not required
    twi_init(&m_twi);

    si7021_init(&m_twi);
    APP_ERROR_CHECK(err_code);
    rtc_config(app_rtc_handler);
    mesh_ttl_set(3);
    while(twi_scan()!= 0x40)
    {
        nrf_delay_ms(100);
    }
    mesh_tx_reset();
    mesh_wait_tx();

    nrf_gpio_pin_clear(29);
    nrf_gpio_pin_clear(28);
   for (uint8_t i = 4; i < 8; i++)
    {
            NRF_POWER->RAM[i].POWER = 0;
            NRF_POWER->RAM[i].POWERCLR = 0x00030003;
    }
    twi_stop();
    clocks_stop(); //release the hf clock
    //NRF_TIMER0-> TASKS_SHUTDOWN = 1;
    while (true)
    {
        nrf_pwr_mgmt_run();
    }
}

