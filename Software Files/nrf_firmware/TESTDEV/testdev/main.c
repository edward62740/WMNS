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
#include "math.h"
//log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//drivers
#include "si1133.h"
#include "veml6030.h"
#include "clocks.h"
#include "battery.h"
#include "twi.h"
#include "mesh.h"
#include "UICR_USER_DEFINES.h"

uint8_t cycle_count = 0;


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
void app_mesh_handler(message_t* msg)
{
    NRF_LOG_INFO("app_mesh_handler()");
}

void app_rtc_handler()
{
    *(volatile uint32_t *)0x40000EE4 = (*(volatile uint32_t *)0x10000258 & 0x0000004F);
    clocks_restart();

nrf_gpio_cfg_output(17);
nrf_gpio_cfg_output(19);
nrf_gpio_cfg_output(20);
    
    if(cycle_count>7) {cycle_count = 0;}
    if(cycle_count == 0) { mesh_tx_reset();nrf_gpio_pin_clear(17); }
    if(cycle_count == 1) { mesh_tx_alive();nrf_gpio_pin_clear(19); }
    if(cycle_count == 2) { mesh_tx_battery(1);nrf_gpio_pin_clear(20); }
    if(cycle_count == 3) { mesh_tx_env(0,1,2,3); nrf_gpio_pin_clear(17);}
    if(cycle_count == 4) { mesh_tx_motion(0,1,2,3,4,5,6); nrf_gpio_pin_clear(19);}
    if(cycle_count == 5) { mesh_tx_iaq(0,1,2,3,4);nrf_gpio_pin_clear(20); }
    if(cycle_count == 6) { mesh_tx_range(0,1,2,3,4); nrf_gpio_pin_clear(17);}
    if(cycle_count == 7) { mesh_tx_co2(0,1,2,3,4);nrf_gpio_pin_clear(19); }
mesh_wait_tx();
 nrf_gpio_pin_set(17);
 nrf_gpio_pin_set(19);
 nrf_gpio_pin_set(20);
nrf_gpio_cfg_default(17);
nrf_gpio_cfg_default(19);
nrf_gpio_cfg_default(20);

    cycle_count++;

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
    clocks_stop();//release the hf clock
    //NRF_TIMER0-> TASKS_SHUTDOWN = 1;
    while(true)
    {
        nrf_pwr_mgmt_run();
    }
}

