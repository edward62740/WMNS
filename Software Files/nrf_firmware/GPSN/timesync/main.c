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
#include "uicr_user_defines.h"
//log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//drivers
#include "clocks.h"
#include "battery.h"
#include "mesh.h"



void app_mesh_handler(message_t* msg)
{
    NRF_LOG_INFO("app_mesh_handler()");
}

void app_rtc_handler()
{
    *(volatile uint32_t *)0x40000EE4 = (*(volatile uint32_t *)0x10000258 & 0x0000004F);
    clocks_restart();
    nrf_gpio_pin_set(4);

    mesh_tx_alive();
   // mesh_tx_range(8989.898,8989.898,8989.898,89.898,8989.898,89.898);
    mesh_wait_tx();
nrf_gpio_pin_clear(4);

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
nrf_gpio_cfg_output(4);
    //NRF_POWER->DCDCEN = 1;
    clocks_start();
    battery_init();
    err_code = mesh_init(app_mesh_handler,NULL);//in case of rf rx, cmd handler not required
    APP_ERROR_CHECK(err_code);
    rtc_config(app_rtc_handler);
    mesh_ttl_set(3);



    mesh_tx_reset();
    mesh_wait_tx();
    clocks_stop();//release the hf clock
    //NRF_TIMER0-> TASKS_SHUTDOWN = 1;
    while(true)
    {
        nrf_pwr_mgmt_run();
    }
}

