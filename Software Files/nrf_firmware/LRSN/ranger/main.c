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
#include "nrf_drv_twi.h"
#include "nrf_drv_rtc.h"
#include "nrf_pwr_mgmt.h"
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
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

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
uint16_t dev = 0x52;

uint16_t WINDOW_LOW = 0;
uint16_t WINDOW_HIGH = 0;
uint16_t WINDOW_AVG = 0;
void recalibration();

void app_mesh_handler(message_t* msg)
{
    NRF_LOG_INFO("app_mesh_handler()");
}

void GPIOTE_IRQHandler(void)
{
    clocks_restart();
    twi_restart();
VL53L1X_ClearInterrupt(dev);

    NVIC_DisableIRQ(GPIOTE_IRQn);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    if(NRF_GPIOTE->EVENTS_PORT)
    {
        NRF_GPIOTE->EVENTS_PORT = 0;

        uint8_t range_stat=0;
        VL53L1X_GetRangeStatus(dev, &range_stat);
        if(range_stat == 0)
        {

            uint16_t dataReady;
            VL53L1X_GetDistance(dev, &dataReady);
            mesh_tx_range(dataReady, 0, 0, 3);
            mesh_wait_tx();

        }
    }
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt
    NVIC_EnableIRQ(GPIOTE_IRQn);
    twi_stop();
    clocks_stop();//release the hf clock

}

void app_rtc_handler()
{

    *(volatile uint32_t *)0x40000EE4 = (*(volatile uint32_t *)0x10000258 & 0x0000004F);
    clocks_restart();
    twi_restart();
    mesh_tx_alive();
    mesh_wait_tx();


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
    for(int i = 0; i < 32; i++)
    {
        nrf_gpio_cfg_default(i);
    }
    //NRF_POWER->DCDCEN = 1;
    clocks_start();
    battery_init();
    err_code = mesh_init(app_mesh_handler,NULL);//in case of rf rx, cmd handler not required
    APP_ERROR_CHECK(err_code);
    twi_init(&m_twi);

    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->EVENTS_PORT = 0;
    nrf_gpio_cfg_sense_input(16, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt



    rtc_config(app_rtc_handler);
    mesh_ttl_set(3);
    //nrf_gpio_cfg_output(15); //shutdown
    //nrf_gpio_cfg_output(14); //interrupt
    nrf_gpio_cfg_input(12, NRF_GPIO_PIN_NOPULL); //mode
    //nrf_gpio_cfg_input(11, NRF_GPIO_PIN_NOPULL); //cal

    nrf_gpio_pin_set(15);

    mesh_tx_reset();
    mesh_wait_tx();

    VL53L1(&m_twi);

    VL53L1X_SensorInit(dev);

       if (nrf_gpio_pin_read(12))
    {
        VL53L1X_SetDistanceMode(dev, 2);
    }
    else
    {
        VL53L1X_SetDistanceMode(dev, 1);
    }

    VL53L1X_SetTimingBudgetInMs(dev, UICR_TIMING_BUDGET_MS);
    VL53L1X_SetInterMeasurementInMs(dev, UICR_INTER_MEAS_INTV_MS);
    VL53L1X_SetDistanceThreshold(dev,1,10,1,1);
    VL53L1X_SetInterruptPolarity(dev, 0);
    VL53L1X_StartRanging(dev);


    //mesh_tx_reset();
    //mesh_wait_tx();

    twi_stop();
    clocks_stop();//release the hf clock
    NVIC_EnableIRQ(GPIOTE_IRQn);
    //NRF_TIMER0-> TASKS_SHUTDOWN = 1;
    while(true)
    {
        nrf_pwr_mgmt_run();
    }
}

