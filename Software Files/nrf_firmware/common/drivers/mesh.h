#ifndef __MESH_H__
#define __MESH_H__
#include <stdint.h>
#include <stdbool.h>

//should match NRF_ESB_MAX_PAYLOAD_LENGTH
#define MAX_MESH_MESSAGE_SIZE (NRF_ESB_MAX_PAYLOAD_LENGTH-4)

typedef struct
{
    uint8_t control;
    uint8_t pid;
    uint16_t source;
    uint16_t dest;
    int8_t  rssi;
    uint8_t payload_length;
    uint8_t *payload;
}message_t;

extern const char * const pid_name[];

//------------------------- Mesh Functions Identifiers -------------------------

// Status processes
#define MESH_PID_RESET         0x00
#define MESH_PID_ALIVE         0x01
#define MESH_PID_BATTERY       0x02

// Controller processes
#define MESH_PID_TRIGGER       0xA3

// End device processes
#define MESH_PID_ENV           0xB0
#define MESH_PID_MOTION        0xB1
#define MESH_PID_VOC           0xB2
#define MESH_PID_RANGE         0xB3
#define MESH_PID_CO2           0xB4
#define MESH_PID_ALS           0xB5

// Router processes
#define MESH_PID_NETSTAT       0xB6

#define MESH_PID_TEXT          0xC0

#define MESH_PID_EXE           0xEC

#define MESH_PID_SYNC          0xF0

#define MESH_Broadcast_Header_Length 5
#define MESH_P2P_Header_Length 7



//------------------------- Mesh Macros -------------------------

#define MESH_IS_BROADCAST(val) ((val & 0x80) == 0x80)
#define MESH_IS_PEER2PEER(val) ((val & 0x80) == 0x00)
//Ack if bits 1,2 == 1,0 => MASK 0x60, VAL 0x40
#define MESH_IS_ACKNOWLEDGE(val) ((val & 0x60) == 0x40)
#define MESH_WANT_ACKNOWLEDGE(val) ((val & 0xF0) == 0x70)
#define MESH_IS_RESPONSE(val) ((val & 0xF0) == 0x00)


//------------------------- Mesh Core -------------------------

typedef void (*app_mesh_rf_handler_t)(message_t*);

typedef void (*app_mesh_cmd_handler_t)(const char*,uint8_t);

uint8_t mesh_channel();
uint8_t mesh_get_channel();
bool mesh_set_crc(uint8_t crc);
uint8_t mesh_get_crc();


uint32_t mesh_init(app_mesh_rf_handler_t rf_handler,app_mesh_cmd_handler_t cmd_handler);

void mesh_execute_cmd(uint8_t*data,uint8_t size,bool is_rf_request,uint8_t rf_nodeid);

//------------------------- Mesh protocol -------------------------

void mesh_consume_rx_messages();

void mesh_wait_tx();
void mesh_tx_message(message_t* msg);
void mesh_ucast_data(uint8_t pid,uint16_t dest,uint8_t * data,uint8_t size);
void mesh_bcast_data(uint8_t pid,uint8_t * data,uint8_t size);
void mesh_bcast_text(char *text);
void mesh_ttl_set(uint8_t ttl);

//------------------------- Send data -------------------------
void mesh_tx_raw(uint8_t* p_data,uint8_t size);
void mesh_tx_reset();
uint32_t mesh_tx_alive();
void mesh_tx_env(int32_t temp, uint32_t hum, uint32_t press, uint32_t light);
void mesh_tx_motion(int32_t x, int32_t y, int32_t z, uint32_t shock_dur, uint32_t shock_count, uint32_t step_count, int16_t sense_temp);
void mesh_tx_battery(uint16_t voltage);
void mesh_tx_voc(int32_t temp, uint32_t hum, uint32_t press, uint32_t light, uint32_t voc);
void mesh_tx_range(uint16_t distance, uint16_t cal_point, uint16_t cal_tolerance, uint8_t range_mode, uint32_t count, uint8_t warning);
void mesh_tx_co2(int32_t temp, uint32_t hum, uint32_t press, uint32_t co2, uint8_t stat);
void mesh_tx_als(uint32_t prials, uint32_t secals, uint32_t w, uint16_t uvi);

//-------------------------- Receive data -------------------------------
void mesh_parse(message_t* msg,char * p_msg);
void mesh_parse_raw(message_t* msg,char * p_msg);
void mesh_parse_bytes(message_t* msg,char * p_msg);


#define MESH_cmd_node_id_set        0x01
#define MESH_cmd_node_id_get        0x02
#define MESH_cmd_rf_chan_set        0x03
#define MESH_cmd_rf_chan_get        0x04
#define MESH_cmd_tx_power_set       0x05
#define MESH_cmd_tx_power_get       0x06
#define MESH_cmd_bitrate_set        0x07
#define MESH_cmd_bitrate_get        0x08
#define MESH_cmd_crc_set            0x09
#define MESH_cmd_crc_get            0x0A

void mesh_text_request(const char*text,uint8_t length);


#endif /*__MESH_H__*/
