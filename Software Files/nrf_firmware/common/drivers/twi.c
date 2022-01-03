#include "twi.h"

#include "sdk_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "boards.h"


/* TWI instance. */
static const nrf_drv_twi_t *p_twi = NULL;


 /* Number of possible TWI addresses. */
#define TWI_ADDRESSES      127


//TODO optimisation
void twi_init(const nrf_drv_twi_t *l_twi)
{
    ret_code_t err_code;

    p_twi = l_twi;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = I2C_SCL,
       .sda                = I2C_SDA,
       .frequency          = TWI_DEFAULT_CONFIG_FREQUENCY,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(p_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(p_twi);
}

void twi_stop()
{
    nrf_drv_twi_disable(p_twi);
    nrf_drv_twi_uninit(p_twi);
    //nRF52832 Errata [89] TWI: Static 400 uA current while using GPIOTE
    *(volatile uint32_t *)0x40003FFC = 0;
    *(volatile uint32_t *)0x40003FFC;
    *(volatile uint32_t *)0x40003FFC = 1;


}

void twi_restart()
{
    //due to workaround Errata [89] Reconfiguration of TWI is required before next usage
    twi_init(p_twi);
}

uint32_t twi_scan(uint32_t addrs)
{
    ret_code_t err_code;
    uint8_t address;
    int n = 0;
    uint8_t sample_data;
    uint8_t devices[4];

    bool detected_device = false;

    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        err_code = nrf_drv_twi_rx(p_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            devices[n] = address;
            n++;
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
           
        }
        NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
        NRF_LOG_INFO("No device was found.");
        NRF_LOG_FLUSH();
        devices[0] = 0;
    }


     return (uint32_t)((devices[1] << 8) | devices[0]);
}
