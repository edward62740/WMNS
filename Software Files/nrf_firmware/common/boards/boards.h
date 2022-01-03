#ifndef BOARDS_H
#define BOARDS_H

#include "nrf_gpio.h"
#include "nordic_common.h"
#if defined(GPSN)
  #include "GPSN.h"
#elif defined(GATEWAY)
  #include "GN.h"
#elif defined(ROUTER32PA)
  #include "R32PAN.h"
#elif defined(ROUTER40)
  #include "R40N.h"
#elif defined(CONTROLLER)
  #include "CN.h"
#elif defined(CO2SN)
  #include "CO2SN.h"
#elif defined(ALSN)
  #include "ALSN.h"
#elif defined(LRSN)
  #include "LRSN.h"
#elif defined(LTSN)
  #include "LTSN.h"
#else
#error "Board is not defined"
#endif
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif
