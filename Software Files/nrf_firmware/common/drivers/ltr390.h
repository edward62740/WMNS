#ifndef __LTR390_h__
#define __LTR390_h__
#define __LTR390_h__

#include <stdint.h>
#include "nrf_drv_twi.h"

#define LTR390_I2CADDR_DEFAULT 0x53 ///< I2C address
#define LTR390_MAIN_CTRL 0x00       ///< Main control register
#define LTR390_MEAS_RATE 0x04       ///< Resolution and data rate
#define LTR390_GAIN 0x05            ///< ALS and UVS gain range
#define LTR390_PART_ID 0x06         ///< Part id/revision register
#define LTR390_MAIN_STATUS 0x07     ///< Main status register
#define LTR390_ALSDATA 0x0D         ///< ALS data lowest byte
#define LTR390_UVSDATA 0x10         ///< UVS data lowest byte
#define LTR390_INT_CFG 0x19         ///< Interrupt configuration
#define LTR390_INT_PST 0x1A         ///< Interrupt persistance config
#define LTR390_THRESH_UP 0x21       ///< Upper threshold, low byte
#define LTR390_THRESH_LOW 0x24      ///< Lower threshold, low byte

/*!    @brief  Whether we are measuring ambient or UV light  */
typedef enum {
	LTR390_MODE_ALS, LTR390_MODE_UVS,
} ltr390_mode_t;

/*!    @brief  Sensor gain for UV or ALS  */
typedef enum {
	LTR390_GAIN_1 = 0,
	LTR390_GAIN_3,
	LTR390_GAIN_6,
	LTR390_GAIN_9,
	LTR390_GAIN_18,
} ltr390_gain_t;

/*!    @brief Measurement resolution (higher res means slower reads!)  */
typedef enum {
	LTR390_RESOLUTION_20BIT,
	LTR390_RESOLUTION_19BIT,
	LTR390_RESOLUTION_18BIT,
	LTR390_RESOLUTION_17BIT,
	LTR390_RESOLUTION_16BIT,
	LTR390_RESOLUTION_13BIT,
} ltr390_resolution_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            LTR390 UV Sensor
 */

bool ltr390_init(const nrf_drv_twi_t *l_twi);
bool ltr390_reset(void);


#endif
void ltr390_enable(void);
void ltr390_disable(void);
bool ltr390_enabled(void);

void ltr390_setMode(ltr390_mode_t mode);
uint8_t ltr390_getMode(void);

void ltr390_setGain(ltr390_gain_t gain);
uint8_t ltr390_getGain(void);

void ltr390_setResolution(ltr390_resolution_t res);
uint8_t ltr390_getResolution(void);

bool ltr390_newDataAvailable(void);
uint32_t ltr390_readUVS(void);
uint32_t ltr390_readALS(void);

