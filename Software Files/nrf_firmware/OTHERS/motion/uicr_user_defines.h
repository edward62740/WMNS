#ifndef __USER_UICR_DEFINES__
#define __USER_UICR_DEFINES__

#define UICR_NODE_ID         0x3001 // motion sensors: 3000 - 4FFF
#define UICR_RF_CHANNEL      14
#define UICR_SLEEP_SEC       5
#define UICR_is_listening()  0x0000
#define UICR_is_router()     0x0000
#define UICR_is_rf_cmd()     0xBABA
#define UICR_is_rf2uart()    0xBABA
#define UICR_is_uart_cmd()   0xBABA
#define UICR_RTX_Timeout     0
#define UICR_RTX_Max_Timeout 5
#define UICR_RTX_Count       3



#endif /*__USER_UICR_DEFINES__*/