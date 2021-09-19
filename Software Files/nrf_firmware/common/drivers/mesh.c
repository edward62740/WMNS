#include "mesh.h"
#include "utils.h"
#include "sdk_common.h"
#include "sdk_config.h"
#include "boards.h"
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include <stdio.h>

#if defined( __GNUC__ ) && (__LINT__ == 0)
// This is required if one wants to use floating-point values in 'printf'
// (by default this feature is not linked together with newlib-nano).
// Please note, however, that this adds about 13 kB code footprint...
__ASM(".global _printf_float");
#endif

#define NRF_LOG_MODULE_NAME mesh

#if (MESH_CONFIG_LOG_ENABLED == 1)
#define NRF_LOG_LEVEL MESH_CONFIG_LOG_LEVEL
#else //MESH_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //MESH_CONFIG_LOG_ENABLED

#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

const char *const pid_name[] = { "" };

#include "uicr_user_defines.h"

static nrf_esb_config_t nrf_esb_config = NRF_ESB_DEFAULT_CONFIG;
static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00);
static nrf_esb_payload_t rx_payload;
static message_t rx_msg;
static volatile bool esb_completed = false;
static volatile bool esb_tx_complete = false;
static uint8_t g_ttl = 3;

static app_mesh_rf_handler_t m_app_rf_handler;
static uint32_t reset_reason;
static app_mesh_cmd_handler_t m_app_cmd_handler;

//forward internal declarations
void mesh_tx_message(message_t *msg);
uint32_t mesh_tx_ack(message_t *msg, uint8_t ttl);
uint32_t mesh_forward_message(message_t *msg);
bool window_check_retransmit();
//uint8_t cmd_parse_response(char* text,uint8_t*data,uint8_t size);

//-------------------------------------------------------------
//----------------------- Payload Store -----------------------
//-------------------------------------------------------------
#if (MESH_TIMER_ENABLED == 1)
const nrf_drv_timer_t TIMER_ACK = NRF_DRV_TIMER_INSTANCE(MESH_TIMER_INSTANCE);

/**
 * @brief Handler for timer events.
 */
void timer_ack_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
    case NRF_TIMER_EVENT_COMPARE0:
        if(!window_check_retransmit())
        {
            nrf_drv_timer_disable(&TIMER_ACK);
        }
        break;
    default:
        //Do nothing.
        break;
    }
}
#endif /*MESH_TIMER_ENABLED*/

typedef struct {
	bool is_waiting_ack;
	uint32_t timeout;
	uint32_t count;
	nrf_esb_payload_t payload;
} esb_payload_store_t;

#define PAYLOAD_STORE_SIZE 2

static esb_payload_store_t tx_payload_window[PAYLOAD_STORE_SIZE];

/**
 * @brief Get the tx payload object from the Store in case a retransmission might be required
 * Otherwise the global tx_payload if no retransmission required or none available on which case
 * re-transmission would not be performed
 *
 * @param control
 * @return nrf_esb_payload_t*
 */
nrf_esb_payload_t* window_get_payload(uint8_t control) {
	nrf_esb_payload_t *res = NULL;
	if ((UICR_RTX_Timeout != 0xFFFFFFFF) && (UICR_RTX_Count != 0xFFFFFFFF)) {
		if ((UICR_RTX_Timeout != 0) && (UICR_RTX_Count != 0)) {
			if (MESH_WANT_ACKNOWLEDGE(control)) {
				//find one free and assign it
				for (int i = 0; (i < PAYLOAD_STORE_SIZE) && (res == NULL);
						i++) {
					if (!tx_payload_window[i].is_waiting_ack) {
						res = &tx_payload_window[i].payload;
						tx_payload_window[i].is_waiting_ack = true;
						tx_payload_window[i].timeout = UICR_RTX_Timeout;
						tx_payload_window[i].count = UICR_RTX_Count;
					}
				}
			}
		}
	}

	if (res == NULL) //in both fails from control test or no store available test
			{
		res = &tx_payload;
	} else {
#if(MESH_TIMER_ENABLED == 1)
        nrf_drv_timer_enable(&TIMER_ACK);
#endif /*MESH_TIMER_ENABLED*/
	}

	return res;
}

bool is_matching_msg_ack(message_t *msg, nrf_esb_payload_t *p_payload) {
	if (msg->pid != p_payload->data[2]) //0:Length, 1:Ctrl, 2:pid
			{
		return false;
	}
	if (msg->source != p_payload->data[5]) //compare source with dest (dest is @ 4)
			{
		return false;
	}
	if (msg->dest != p_payload->data[3]) //compare dest with source (source is @ 3)
			{
		return false;
	}
	return true;
}

void window_remove_payload(message_t *msg) {
	bool found = false;
	for (int i = 0; (i < PAYLOAD_STORE_SIZE) && (!found); i++) {
		if (tx_payload_window[i].is_waiting_ack) {
			if (is_matching_msg_ack(msg, &tx_payload_window[i].payload)) {
				//TODO if serial cmd request, should report the rtx count
				tx_payload_window[i].is_waiting_ack = false;
				found = true;
			}
		}
	}
}

/**
 * @brief
 *
 * @return true the timer is still required
 * @return false no timeout still required
 */
bool window_check_retransmit() {
	bool timer_still_required = false;
	for (int i = 0; i < PAYLOAD_STORE_SIZE; i++) {
		if (tx_payload_window[i].is_waiting_ack) {
			bool this_timeout_is_still_required = true;
			if (--tx_payload_window[i].timeout == 0) {
				//retransmit
				nrf_esb_write_payload(&tx_payload_window[i].payload);
				if (--tx_payload_window[i].count == 0) {
					//retransmit count over !!!!! free the slot
					tx_payload_window[i].is_waiting_ack = false;
					this_timeout_is_still_required = false;
				}
				//restart a new timeout for the next count
				tx_payload_window[i].timeout = UICR_RTX_Timeout;
			}
			if (this_timeout_is_still_required) {
				timer_still_required = true;
			}
		}
	}
	return timer_still_required;
}

//-------------------------------------------------------------
//------------------------- Mesh Core -------------------------
//-------------------------------------------------------------

uint8_t mesh_channel() {
	return UICR_RF_CHANNEL;
}

uint8_t mesh_get_channel() {
	return NRF_RADIO->FREQUENCY;
}

bool mesh_set_crc(uint8_t crc) {
	switch (crc) {
	case NRF_ESB_CRC_16BIT:
		NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
		NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
		break;

	case NRF_ESB_CRC_8BIT:
		NRF_RADIO->CRCINIT = 0xFFUL;        // Initial value
		NRF_RADIO->CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
		break;

	case NRF_ESB_CRC_OFF:
		break;

	default:
		return false;
	}
	NRF_RADIO->CRCCNF = crc;
	return true;
}

uint8_t mesh_get_crc() {
	return (uint8_t) NRF_RADIO->CRCCNF;
}

void mesh_pre_tx() {
	if (UICR_is_listening()) {
		nrf_esb_stop_rx();
		NRF_LOG_DEBUG("switch to IDLE mode that allows TX");
	}

}

void mesh_post_tx() {
	if (UICR_is_listening()) {
		nrf_esb_start_rx();
		NRF_LOG_DEBUG("switch to RX mode");
	}
}

void mesh_message_2_esb_payload(message_t *msg,
		nrf_esb_payload_t *p_tx_payload) {
	//esb only parameters
	p_tx_payload->noack = true;       //Never request an ESB acknowledge
	p_tx_payload->pipe = 0;       //pipe is the selection of the address to use

	p_tx_payload->data[1] = msg->control;
	p_tx_payload->data[2] = msg->pid;
	p_tx_payload->data[3] = msg->source;
	p_tx_payload->data[4] = msg->source >> 8;

	uint8_t start_payload;
	if (MESH_IS_BROADCAST(msg->control)) {
		start_payload = MESH_Broadcast_Header_Length;
	} else {
		p_tx_payload->data[5] = msg->dest;
		p_tx_payload->data[6] = msg->dest >> 8;
		start_payload = MESH_P2P_Header_Length;
	}

	//this is the total ESB packet length
	p_tx_payload->length = msg->payload_length + start_payload;
	p_tx_payload->data[0] = p_tx_payload->length;
	memcpy(p_tx_payload->data + start_payload, msg->payload,
			msg->payload_length);
}

void mesh_esb_2_message_payload(nrf_esb_payload_t *p_rx_payload,
		message_t *msg) {
	msg->control = p_rx_payload->data[1];
	msg->pid = p_rx_payload->data[2];
	msg->source = p_rx_payload->data[3];
	msg->source |= p_rx_payload->data[4] << 8;
	msg->rssi = p_rx_payload->rssi;
	//dest is processed by esb rx function
	uint8_t payload_start;
	if (MESH_IS_BROADCAST(msg->control)) {
		msg->dest = 65535;
		payload_start = 5;
	} else {
		msg->dest = p_rx_payload->data[5];
		msg->dest |= p_rx_payload->data[6] << 8;
		payload_start = 7;
	}
	msg->payload_length = p_rx_payload->length - payload_start;

	if (msg->payload_length > 0) {
		msg->payload = p_rx_payload->data + payload_start;
	}
}

void mesh_rx_handler(message_t *msg) {
	//The app gets everything - and at top for time sync
	if (m_app_rf_handler != NULL) {
		m_app_rf_handler(msg);
	}

	bool is_to_be_forwarded = false;
	if (msg->dest == UICR_NODE_ID)       //current node id match
			{
		if (MESH_WANT_ACKNOWLEDGE(msg->control)) {
			mesh_tx_ack(msg, 2);
		} else if (MESH_IS_ACKNOWLEDGE(msg->control)) {
			window_remove_payload(msg); //rx_payload is a global shared variable uniquely used during reception
		}
	}
	//only re-route messages directed to other than the current node itself
	else if (UICR_is_router()) {
		is_to_be_forwarded = true;
	}

	//as the message forward is destructive it is to be done at the last step
	if (is_to_be_forwarded) {
		mesh_forward_message(msg);
	}
}

void mesh_consume_rx_messages() {
	static uint32_t count = 0;
	// Get the most recent element from the RX FIFO.
	while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS) {
		mesh_esb_2_message_payload(&rx_payload, &rx_msg);
		NRF_LOG_INFO("ESB Rx %d- pipe: (%d) -> pid:%d ; length:%d", count++,
				rx_payload.pipe, rx_payload.pid, rx_payload.length);
		NRF_LOG_INFO("HSM - src: (%d) -> pid:0x%02X ; length:%d", rx_msg.source,
				rx_msg.pid, rx_msg.payload_length);
		mesh_rx_handler(&rx_msg);
	}
}

void nrf_esb_event_handler(nrf_esb_evt_t const *p_event) {
	switch (p_event->evt_id) {
	case NRF_ESB_EVENT_TX_SUCCESS:
		NRF_LOG_DEBUG("ESB TX SUCCESS EVENT");
		mesh_post_tx();
		esb_tx_complete = true;
		break;
	case NRF_ESB_EVENT_TX_FAILED:
		NRF_LOG_DEBUG("ESB TX FAILED EVENT");
		(void) nrf_esb_flush_tx();
		mesh_post_tx();
		esb_tx_complete = true;
		break;
	case NRF_ESB_EVENT_RX_RECEIVED:
		NRF_LOG_DEBUG("________________ESB RX RECEIVED EVENT________________");
		//Do nothing, handled in while loop
		//but return immidiatly so that more rx events are filled in the rx fifo
		break;
	default:
		esb_completed = true;
		NRF_LOG_ERROR("ESB Unhandled Event (%d)", p_event->evt_id);
		break;
	}

	esb_completed = true;
}

uint32_t mesh_init(app_mesh_rf_handler_t rf_handler,
		app_mesh_cmd_handler_t cmd_handler) {
	uint32_t err_code;
	uint8_t base_addr_0[4] = { 0xE7, 0xE7, 0xE7, 0xE7 };
	uint8_t base_addr_1[4] = { 0xC2, 0xC2, 0xC2, 0xC2 };
	uint8_t addr_prefix[8] = { 0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

	m_app_rf_handler = rf_handler;
	m_app_cmd_handler = cmd_handler;

	nrf_esb_config.retransmit_count = 0;
	nrf_esb_config.selective_auto_ack = true; //false is not supported : payload.noack  decides
	nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB_DPL;
	nrf_esb_config.payload_length = 8; //Not used by DPL as then the MAX is configured
	nrf_esb_config.bitrate = NRF_ESB_BITRATE_1MBPS_BLE;
	nrf_esb_config.event_handler = nrf_esb_event_handler;
	if (UICR_is_listening()) {
		nrf_esb_config.mode = NRF_ESB_MODE_PRX;
	} else {
		nrf_esb_config.mode = NRF_ESB_MODE_PTX;
	}

	nrf_esb_config.crc = NRF_ESB_CRC_16BIT;

	err_code = nrf_esb_init(&nrf_esb_config);
	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_base_address_0(base_addr_0);
	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_base_address_1(base_addr_1);
	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_prefixes(addr_prefix, 8);
	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_rf_channel(UICR_RF_CHANNEL);
	VERIFY_SUCCESS(err_code);

#if defined(ROUTER32PA) || (GATEWAY)
    err_code = nrf_esb_set_tx_power(RADIO_TXPOWER_TXPOWER_Pos4dBm);
    VERIFY_SUCCESS(err_code);
#endif

	tx_payload.length = 8;
	tx_payload.pipe = 0;
	tx_payload.data[0] = 0x00;

	NRF_LOG_INFO("nodeId %d", UICR_NODE_ID);
	NRF_LOG_INFO("channel %d", UICR_RF_CHANNEL);

#if (MESH_TIMER_ENABLED == 1)
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_ACK, &timer_cfg, timer_ack_event_handler);
    VERIFY_SUCCESS(err_code);

    uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_ACK, 1);//1 ms

    nrf_drv_timer_extended_compare(
        &TIMER_ACK, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_ACK);
#endif/*MESH_TIMER_ENABLED*/

	if (UICR_is_listening()) {
		err_code = nrf_esb_start_rx();
		VERIFY_SUCCESS(err_code);
		NRF_LOG_INFO("UICR listening");
	} else {
		NRF_LOG_INFO("UICR Not listening");
	}

	reset_reason = 0xF000F & NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = 0xF000F & 0xFFFFF; //reset register

	return NRF_SUCCESS;
}

/**
 * @brief can be used before going into sleep as RADIO does not operate in low power
 * This will return either after success or failure event
 */
void mesh_wait_tx() {
	while (!esb_completed);
}

//--------------------------------------------------------------------------
//------------------------- Messages Serialisation -------------------------
//--------------------------------------------------------------------------

/**
 * @brief required TX raw as it prevents partial copies before using the same mesh_tx_message
 * Note that the length is not expected to be the first byte here as it will be added
 *
 * @param p_data
 * @param size
 */
void mesh_tx_raw(uint8_t *p_data, uint8_t size) {
	mesh_pre_tx();

	nrf_esb_payload_t *p_tx_payload = window_get_payload(p_data[0]); //Starts with control as first byte
	p_tx_payload->length = size + 1;        //as size itself is now added
	p_tx_payload->data[0] = p_tx_payload->length;
	memcpy(p_tx_payload->data + 1, p_data, size);

	esb_completed = false;        //reset the check
	nrf_esb_write_payload(p_tx_payload);
}
/**
 * @brief Transmits a message structure
 *
 * @param msg : the message structure to be converted to an esb buffer and sent through nrf_esb_write_payload
 */
void mesh_tx_message(message_t *p_msg) {
	mesh_pre_tx();

	nrf_esb_payload_t *p_tx_payload = window_get_payload(p_msg->control);
	mesh_message_2_esb_payload(p_msg, p_tx_payload);

	esb_completed = false;        //reset the check
	NRF_LOG_DEBUG("________________TX esb payload length = %d________________",
			p_tx_payload->data[0]);
	//should not wait for esb_completed here as does not work from ISR context

	//NRF_ESB_TXMODE_AUTO is used no need to call nrf_esb_start_tx()
	//which could be used for a precise time transmission
	nrf_esb_write_payload(p_tx_payload);

	//nrf_esb_start_tx();//as the previous mode does not start it due to .mode
}

uint32_t mesh_tx_ack(message_t *msg, uint8_t ttl) {
	message_t ack;

	ack.control = 0x40 | ttl;         // ack | ttl = 2
	ack.pid = msg->pid;
	ack.source = msg->dest;        // == UICR_NODE_ID
	ack.dest = msg->source;
	ack.payload_length = 0;

	mesh_tx_message(&ack);

	return NRF_SUCCESS;
}

void alive_add_rtx_info(message_t *msg) {
	uint8_t add_index = msg->payload_length;
	msg->payload[add_index++] = msg->rssi;
	msg->payload[add_index++] = get_this_node_id();
	msg->payload[add_index++] = NRF_RADIO->TXPOWER;
	msg->payload_length += 3;
}

uint32_t mesh_forward_message(message_t *msg) {
	uint8_t ttl = msg->control & 0x0F;
	if (ttl > 0) {
		//Add TTL
		ttl--;
		msg->control &= 0xF0;        //clear ttl
		msg->control |= ttl;        //set new ttl
		//Update Alive
		if (msg->pid == MESH_PID_ALIVE)    //rework to add the new (rssi,nid,tx)
				{
			alive_add_rtx_info(msg);
		}
		//send it
		mesh_tx_message(msg);
	}

	return NRF_SUCCESS;
}

/**
 * @brief Sends a simple message that has no payload, by providing only the pid
 *
 * @param pid the application protocol id : e.g. reset,...
 * @return uint32_t
 */
uint32_t mesh_tx_pid(uint8_t pid) {
	message_t msg;

	msg.control = 0x80 | 2;         // broadcast | ttl = 2
	msg.pid = pid;
	msg.source = UICR_NODE_ID;
	msg.payload_length = 0;

	mesh_tx_message(&msg);

	return NRF_SUCCESS;
}

/**
 * @brief Sends a reset message, used usually when a device wakes up from reset
 * Combined in a database, it keeps history of when the device was first started
 * Of whether the device has reset due to any error
 *
 */
void mesh_tx_reset() {
	uint8_t data[4];
	data[0] = 0xFF & (uint8_t)(reset_reason >> 24);
	data[1] = 0xFF & (uint8_t)(reset_reason >> 16);
	data[2] = 0xFF & (uint8_t)(reset_reason >> 8);
	data[3] = 0xFF & (uint8_t)(reset_reason);
	mesh_bcast_data(MESH_PID_RESET, data, 4);
}

void mesh_ttl_set(uint8_t ttl) {
	g_ttl = ttl;
}

void mesh_ucast_data(uint8_t pid, uint16_t dest, uint8_t *data, uint8_t size) {
	message_t msg;

	msg.control = 0x00 | g_ttl;         // response | ttl = g_ttl
	msg.pid = pid;
	msg.source = UICR_NODE_ID;
	msg.dest = dest;
	msg.payload = data;
	msg.payload_length = size;

	mesh_tx_message(&msg);
}

void mesh_bcast_data(uint8_t pid, uint8_t *data, uint8_t size) {
	message_t msg;

	msg.control = 0x80 | g_ttl;         // broadcast | ttl = g_ttl
	msg.pid = pid;
	msg.source = UICR_NODE_ID;
	msg.payload = data;
	msg.payload_length = size;

	mesh_tx_message(&msg);
}

//limited to 255
void mesh_bcast_text(char *text) {
	uint8_t size = strlen(text);
	if (size > MAX_MESH_MESSAGE_SIZE)         //truncate in case of long message
			{
		text[MAX_MESH_MESSAGE_SIZE - 1] = '>';
		size = MAX_MESH_MESSAGE_SIZE;
	}
	mesh_bcast_data(MESH_PID_TEXT, (uint8_t*) text, size);
}

/**
 * @brief Broadcast an alive packet with associated payload information
 * The payload contains a livecounter (uint32_t) and the RF transmission power (int8_t)
 *
 */
uint32_t mesh_tx_alive() {
	static uint32_t live_count = 0;

	uint8_t data[5];
	data[0] = 0xFF & (uint8_t)(live_count >> 24);
	data[1] = 0xFF & (uint8_t)(live_count >> 16);
	data[2] = 0xFF & (uint8_t)(live_count >> 8);
	data[3] = 0xFF & (uint8_t)(live_count);
	data[4] = NRF_RADIO->TXPOWER;
#if defined(ROUTER32PA)
    data[4] = (uint8_t)22;
#endif

	mesh_bcast_data(MESH_PID_ALIVE, data, 5);

	return live_count++;     //returns the first used value before the increment
}

void mesh_tx_env(int32_t temp, uint32_t hum, uint32_t press, uint32_t light) {
	uint8_t data[16];
	data[0] = 0xFF & (uint8_t)(temp >> 24);
	data[1] = 0xFF & (uint8_t)(temp >> 16);
	data[2] = 0xFF & (uint8_t)(temp >> 8);
	data[3] = 0xFF & (uint8_t)(temp);
	data[4] = 0xFF & (uint8_t)(hum >> 24);
	data[5] = 0xFF & (uint8_t)(hum >> 16);
	data[6] = 0xFF & (uint8_t)(hum >> 8);
	data[7] = 0xFF & (uint8_t)(hum);
	data[8] = 0xFF & (uint8_t)(press >> 24);
	data[9] = 0xFF & (uint8_t)(press >> 16);
	data[10] = 0xFF & (uint8_t)(press >> 8);
	data[11] = 0xFF & (uint8_t)(press);
	data[12] = 0xFF & (uint8_t)(light >> 24);
	data[13] = 0xFF & (uint8_t)(light >> 16);
	data[14] = 0xFF & (uint8_t)(light >> 8);
	data[15] = 0xFF & (uint8_t)(light);
	mesh_bcast_data(MESH_PID_ENV, data, 16);
}

void mesh_tx_motion(int32_t x, int32_t y, int32_t z, uint32_t shock_dur,
		uint32_t shock_count, uint32_t step_count, int16_t sense_temp) {

	uint8_t data[26];
	data[0] = 0xFF & (uint8_t)(x >> 24);
	data[1] = 0xFF & (uint8_t)(x >> 16);
	data[2] = 0xFF & (uint8_t)(x >> 8);
	data[3] = 0xFF & (uint8_t)(x);
	data[4] = 0xFF & (uint8_t)(y >> 24);
	data[5] = 0xFF & (uint8_t)(y >> 16);
	data[6] = 0xFF & (uint8_t)(y >> 8);
	data[7] = 0xFF & (uint8_t)(y);
	data[8] = 0xFF & (uint8_t)(z >> 24);
	data[9] = 0xFF & (uint8_t)(z >> 16);
	data[10] = 0xFF & (uint8_t)(z >> 8);
	data[11] = 0xFF & (uint8_t)(z);
	data[12] = 0xFF & (uint8_t)(shock_dur >> 24);
	data[13] = 0xFF & (uint8_t)(shock_dur >> 16);
	data[14] = 0xFF & (uint8_t)(shock_dur >> 8);
	data[15] = 0xFF & (uint8_t)(shock_dur);
	data[16] = 0xFF & (uint8_t)(shock_count >> 24);
	data[17] = 0xFF & (uint8_t)(shock_count >> 16);
	data[18] = 0xFF & (uint8_t)(shock_count >> 8);
	data[19] = 0xFF & (uint8_t)(shock_count);
	data[20] = 0xFF & (uint8_t)(step_count >> 24);
	data[21] = 0xFF & (uint8_t)(step_count >> 16);
	data[22] = 0xFF & (uint8_t)(step_count >> 8);
	data[23] = 0xFF & (uint8_t)(step_count);
	data[24] = 0xFF & (uint8_t)(sense_temp >> 8);
	data[25] = 0xFF & (uint8_t)(sense_temp);

	mesh_bcast_data(MESH_PID_MOTION, data, 26);
}

void mesh_tx_voc(int32_t temp, uint32_t hum, uint32_t press, uint32_t light,
		uint32_t voc) {
	uint8_t data[20];
	data[0] = 0xFF & (uint8_t)(temp >> 24);
	data[1] = 0xFF & (uint8_t)(temp >> 16);
	data[2] = 0xFF & (uint8_t)(temp >> 8);
	data[3] = 0xFF & (uint8_t)(temp);
	data[4] = 0xFF & (uint8_t)(hum >> 24);
	data[5] = 0xFF & (uint8_t)(hum >> 16);
	data[6] = 0xFF & (uint8_t)(hum >> 8);
	data[7] = 0xFF & (uint8_t)(hum);
	data[8] = 0xFF & (uint8_t)(press >> 24);
	data[9] = 0xFF & (uint8_t)(press >> 16);
	data[10] = 0xFF & (uint8_t)(press >> 8);
	data[11] = 0xFF & (uint8_t)(press);
	data[12] = 0xFF & (uint8_t)(light >> 24);
	data[13] = 0xFF & (uint8_t)(light >> 16);
	data[14] = 0xFF & (uint8_t)(light >> 8);
	data[15] = 0xFF & (uint8_t)(light);
	data[16] = 0xFF & (uint8_t)(voc >> 24);
	data[17] = 0xFF & (uint8_t)(voc >> 16);
	data[18] = 0xFF & (uint8_t)(voc >> 8);
	data[19] = 0xFF & (uint8_t)(voc);
	mesh_bcast_data(MESH_PID_VOC, data, 20);
}

void mesh_tx_range(uint16_t distance, uint16_t cal_point,
		uint16_t cal_tolerance, uint8_t range_mode, uint32_t count, uint8_t warning) {
	uint8_t data[12];
	data[0] = 0xFF & (uint8_t)(distance >> 8);
	data[1] = 0xFF & (uint8_t)(distance);
	data[2] = 0xFF & (uint8_t)(cal_point >> 8);
	data[3] = 0xFF & (uint8_t)(cal_point);
	data[4] = 0xFF & (uint8_t)(cal_tolerance >> 8);
	data[5] = 0xFF & (uint8_t)(cal_tolerance);
	data[6] = 0xFF & (uint8_t)(range_mode);
	data[7] = 0xFF & (uint8_t)(count >> 24);
	data[8] = 0xFF & (uint8_t)(count >> 16);
	data[9] = 0xFF & (uint8_t)(count >> 8);
	data[10] = 0xFF & (uint8_t)(count);
	data[11] = 0xFF & (uint8_t)(warning);
	mesh_bcast_data(MESH_PID_RANGE, data, 12);
}

void mesh_tx_co2(int32_t temp, uint32_t hum, uint32_t press, uint32_t co2,
		uint8_t stat) {
	uint8_t data[17];
	data[0] = 0xFF & (uint8_t)(temp >> 24);
	data[1] = 0xFF & (uint8_t)(temp >> 16);
	data[2] = 0xFF & (uint8_t)(temp >> 8);
	data[3] = 0xFF & (uint8_t)(temp);
	data[4] = 0xFF & (uint8_t)(hum >> 24);
	data[5] = 0xFF & (uint8_t)(hum >> 16);
	data[6] = 0xFF & (uint8_t)(hum >> 8);
	data[7] = 0xFF & (uint8_t)(hum);
	data[8] = 0xFF & (uint8_t)(press >> 24);
	data[9] = 0xFF & (uint8_t)(press >> 16);
	data[10] = 0xFF & (uint8_t)(press >> 8);
	data[11] = 0xFF & (uint8_t)(press);
	data[12] = 0xFF & (uint8_t)(co2 >> 24);
	data[13] = 0xFF & (uint8_t)(co2 >> 16);
	data[14] = 0xFF & (uint8_t)(co2 >> 8);
	data[15] = 0xFF & (uint8_t)(co2);
	data[16] = 0xFF & (uint8_t)(stat);
	mesh_bcast_data(MESH_PID_CO2, data, 17);
}

void mesh_tx_als(uint32_t prials, uint32_t secals, uint32_t w, uint16_t uvi) {
	uint8_t data[14];
	data[0] = 0xFF & (uint8_t)(prials >> 24);
	data[1] = 0xFF & (uint8_t)(prials >> 16);
	data[2] = 0xFF & (uint8_t)(prials >> 8);
	data[3] = 0xFF & (uint8_t)(prials);
	data[4] = 0xFF & (uint8_t)(secals >> 24);
	data[5] = 0xFF & (uint8_t)(secals >> 16);
	data[6] = 0xFF & (uint8_t)(secals >> 8);
	data[7] = 0xFF & (uint8_t)(secals);
	data[8] = 0xFF & (uint8_t)(w >> 24);
	data[9] = 0xFF & (uint8_t)(w >> 16);
	data[10] = 0xFF & (uint8_t)(w >> 8);
	data[11] = 0xFF & (uint8_t)(w);
	data[12] = 0xFF & (uint8_t)(uvi << 8);
	data[13] = 0xFF & (uint8_t)(uvi);
	mesh_bcast_data(MESH_PID_ALS, data, 14);
}

void mesh_tx_battery(uint16_t voltage) {
	uint8_t data[2];
	data[0] = 0xFF & (uint8_t)(voltage >> 8);
	data[1] = 0xFF & (uint8_t)(voltage);
	mesh_bcast_data(MESH_PID_BATTERY, data, 2);
}

//----------------------------------------------------------------------------
//------------------------- Messages Deserialisation -------------------------
//----------------------------------------------------------------------------

int rx_alive(char *p_msg, uint8_t *data, uint8_t size, int8_t rssi) {
	if (size == 0) {
		return sprintf(p_msg, "alive:1");
	} else if ((size >= 5) && ((size - 5) % 3 == 0)) //size pattern of alive messages
			{
		//-------------------------- 3 sections --------------------------
		uint32_t live_count;
		int data_index = 0;
		live_count = data[data_index++] << 24;
		live_count |= data[data_index++] << 16;
		live_count |= data[data_index++] << 8;
		live_count |= data[data_index++];
		//-------------------------- section one permanent --------------------------
		int nb_rtx = (size - 5) / 3;     //must be rounded here as % test passed
		int add = sprintf(p_msg, "alive:%lu;nb:%u", live_count, nb_rtx + 1);
		p_msg += add;
		int total_print = add;
		//----------------------- section two depends on nb rtx -----------------------
		for (int i = 0; i < nb_rtx; i++) {
			add = sprintf(p_msg, ";rx%d:%d,-%d,%d", i + 1, data[data_index],
					data[data_index + 1], data[data_index + 2]);
			data_index += 3; //to avoid the error of operation may be undefined with muliple increments in sprintf
			p_msg += add;
			total_print += add;
		}
		//----------------------- section three permanent, the last rtx is special -----------------------
		add = sprintf(p_msg, ";rx%d:%d,-%d,%d", nb_rtx + 1, data[data_index++],
				rssi, get_this_node_id());
		total_print += add;
		return total_print;
	} else {
		return sprintf(p_msg, "size not 0, not 5 but:%d", size);
	}
}
int rx_reset(char *p_msg, uint8_t *data, uint8_t size) {
	if (size == 4) {
		uint32_t rst = data[3];
		rst |= data[2] << 8;
		rst |= data[1] << 16;
		rst |= data[0] << 24;
		return sprintf(p_msg, "reset:%lu", rst);
	} else {
		return sprintf(p_msg, "size not 4 but:%d", size);
	}}
int rx_env(char *p_msg, uint8_t *data, uint8_t size) {
	char *p_start = p_msg;
	if (size != 16) {
		return sprintf(p_msg, "size not 16 but:%u", size);
	} else {
		int32_t temp = data[0] << 24;
		temp |= data[1] << 16;
		temp |= data[2] << 8;
		temp |= data[3];
		//print integers only and (-) handled manually, so turn to positive and force sign character in sprintf()
		if (temp < 0) {
			temp *= (-1);
			p_msg += sprintf(p_msg, "temp:-");
		} else {
			p_msg += sprintf(p_msg, "temp:");
		}
		int32_t mst = temp / 100;
		int32_t lst = temp % 100;
		p_msg += sprintf(p_msg, "%ld.%02ld;", mst, lst);
		uint32_t hum = data[4] << 24;
		hum |= data[5] << 16;
		hum |= data[6] << 8;
		hum |= data[7];
		uint32_t msh = hum >> 10;
		uint32_t lsh = hum & 0x3FF;
		uint32_t press = data[8] << 24;
		press |= data[9] << 16;
		press |= data[10] << 8;
		press |= data[11];
		uint32_t msp = press / (256 * 100);
		uint32_t lsp = (press / 256) % 100;
		p_msg += sprintf(p_msg, "hum:%lu.%03lu;press:%lu.%02lu", msh, lsh, msp,
				lsp);
		uint32_t light = data[15];
		light |= data[14] << 8;
		light |= data[13] << 16;
		light |= data[12] << 24;
		float f_light = light;
		f_light /= 1000;
		//uint32_t msl = press / (4096 * 1000);
		//uint32_t lsl = (press/4096) % 1000;
		p_msg += sprintf(p_msg, ";light:%.03f", f_light);
		return (p_msg - p_start);
	}

}

int rx_voc(char *p_msg, uint8_t *data, uint8_t size) {
	char *p_start = p_msg;
	if (size != 20) {
		return sprintf(p_msg, "size not 20 but:%u", size);
	} else {
		int32_t temp = data[0] << 24;
		temp |= data[1] << 16;
		temp |= data[2] << 8;
		temp |= data[3];
		//print integers only and (-) handled manually, so turn to positive and force sign character in sprintf()
		if (temp < 0) {
			temp *= (-1);
			p_msg += sprintf(p_msg, "temp:-");
		} else {
			p_msg += sprintf(p_msg, "temp:");
		}
		int32_t mst = temp / 100;
		int32_t lst = temp % 100;
		p_msg += sprintf(p_msg, "%ld.%02ld;", mst, lst);
		uint32_t hum = data[4] << 24;
		hum |= data[5] << 16;
		hum |= data[6] << 8;
		hum |= data[7];
		uint32_t msh = hum >> 10;
		uint32_t lsh = hum & 0x3FF;
		uint32_t press = data[8] << 24;
		press |= data[9] << 16;
		press |= data[10] << 8;
		press |= data[11];
		float f_press = press;
		f_press /= 100;
		p_msg += sprintf(p_msg, "hum:%lu.%03lu;press:%.03f", msh, lsh, f_press);
		uint32_t light = data[15];
		light |= data[14] << 8;
		light |= data[13] << 16;
		light |= data[12] << 24;
		float f_light = light;
		f_light /= 1000;
		p_msg += sprintf(p_msg, ";light:%.03f", f_light);

		uint32_t voc = data[19];
		voc |= data[18] << 8;
		voc |= data[17] << 16;
		voc |= data[16] << 24;
		float f_voc = voc;
		f_voc /= 100;

		p_msg += sprintf(p_msg, ";voc:%.02f", f_voc);

		return (p_msg - p_start);
	}
}

int rx_motion(char *p_msg, uint8_t *data, uint8_t size) {
	if (size == 26) {
		int32_t x = data[3];
		x |= data[2] << 8;
		x |= data[1] << 16;
		x |= data[0] << 24;
		float float_x = x;
		float_x /= 1000;

		int32_t y = data[7];
		y |= data[6] << 8;
		y |= data[5] << 16;
		y |= data[4] << 24;
		float float_y = y;
		float_y /= 1000;

		int32_t z = data[11];
		z |= data[10] << 8;
		z |= data[9] << 16;
		z |= data[8] << 24;
		float float_z = z;
		float_z /= 1000;

		uint32_t shock_dur = data[15];
		shock_dur |= data[14] << 8;
		shock_dur |= data[13] << 16;
		shock_dur |= data[12] << 24;
		float float_shock_dur = shock_dur;
		float_shock_dur /= 320000;

		uint32_t shock_count = data[19];
		shock_count |= data[18] << 8;
		shock_count |= data[17] << 16;
		shock_count |= data[16] << 24;
		int shocks = shock_count;

		uint32_t step_count = data[23];
		step_count |= data[22] << 8;
		step_count |= data[21] << 16;
		step_count |= data[20] << 24;

		int steps = step_count;

		int16_t sense_temp = data[25];
		sense_temp |= data[24] << 8;

		int temp = sense_temp;

		return sprintf(p_msg,
				"x:%.03f;y:%.03f;z:%.03f;shockdur:%.03f;shockcount:%.i;stepcount:%.i;sensetemp:%.i",
				float_x, float_y, float_z, float_shock_dur, shocks, steps, temp);
	} else {
		return sprintf(p_msg, "size not 26 but:%d", size);
	}
}

int rx_range(char *p_msg, uint8_t *data, uint8_t size) {
	if (size == 12) {
		uint16_t distance = data[1];
		distance |= data[0] << 8;
		int distance_mm = distance;

		uint16_t cal_point = data[3];
		cal_point |= data[2] << 8;
		int cal_point_mm = cal_point;

		uint16_t cal_tolerance = data[5];		uint32_t light = data[15];
		light |= data[14] << 8;
		light |= data[13] << 16;
		light |= data[12] << 24;
		cal_tolerance |= data[4] << 8;
		int cal_tolerance_mm = cal_tolerance;

		uint8_t range_mode = data[6];
		int range_mode_int = range_mode;

		uint32_t count = data[10];
		count |= data[9] << 8;
		count |= data[8] << 16;
		count |= data[7] << 24;

		uint8_t warning = data[11];

		return sprintf(p_msg, "distance:%.i;cal:%.i;tol:%.i;mode:%.i;count:%lu;warn:%.i",
				distance_mm, cal_point_mm, cal_tolerance_mm, range_mode_int,
				count,warning);
	} else {
		return sprintf(p_msg, "size not 12 but:%d", size);
	}
}

int rx_co2(char *p_msg, uint8_t *data, uint8_t size) {
	char *p_start = p_msg;
	if (size != 17) {
		return sprintf(p_msg, "size not 17 but:%u", size);
	} else {
		int32_t temp = data[0] << 24;
		temp |= data[1] << 16;
		temp |= data[2] << 8;
		temp |= data[3];
		//print integers only and (-) handled manually, so turn to positive and force sign character in sprintf()
		if (temp < 0) {
			temp *= (-1);
			p_msg += sprintf(p_msg, "temp:-");
		} else {
			p_msg += sprintf(p_msg, "temp:");
		}
		int32_t mst = temp / 1000;
		int32_t lst = temp % 1000;
		p_msg += sprintf(p_msg, "%ld.%03ld;", mst, lst);
		uint32_t hum = data[4] << 24;
		hum |= data[5] << 16;
		hum |= data[6] << 8;
		hum |= data[7];
		uint32_t msh = hum >> 10;
		uint32_t lsh = hum & 0x3FF;
		uint32_t press = data[8] << 24;
		press |= data[9] << 16;
		press |= data[10] << 8;
		press |= data[11];
		uint32_t msp = press / (256 * 100);
		uint32_t lsp = (press / 256) % 100;
		p_msg += sprintf(p_msg, "hum:%lu.%03lu;press:%lu.%02lu", msh, lsh, msp,
				lsp);
		uint32_t co2 = data[15];
		co2 |= data[14] << 8;
		co2 |= data[13] << 16;
		co2 |= data[12] << 24;
		float f_co2 = co2;
		p_msg += sprintf(p_msg, ";co2:%.01f", f_co2);

		uint8_t ret = data[16];
		p_msg += sprintf(p_msg, ";status:%d", ret);

		return (p_msg - p_start);
	}

}

int rx_als(char *p_msg, uint8_t *data, uint8_t size) {
	if (size == 14) {
		uint32_t prials = data[3];
		prials |= data[2] << 8;
		prials |= data[1] << 16;
		prials |= data[0] << 24;
		double f_prials = prials;
		f_prials /= 1000;


		uint32_t secals = data[7];
		secals |= data[6] << 8;
		secals |= data[5] << 16;
		secals |= data[4] << 24;
		float f_secals = secals;
		f_secals /= 1000;

		uint32_t w = data[11];
		w |= data[10] << 8;
		w |= data[9] << 16;
		w |= data[8] << 24;
		double f_w = w;
		f_w /= 1000;

		uint16_t uvi = data[13];
		uvi |= data[12] << 8;


		return sprintf(p_msg, "als1:%.03f;als2:%.03f;w:%.03f;uvi:%.i",
				f_prials, f_secals, f_w, uvi);
	} else {
		return sprintf(p_msg, "size not 14 but:%d", size);
	}
}

int rx_battery(char *p_msg, uint8_t *data, uint8_t size) {
	if (size != 2) {
		return sprintf(p_msg, "size not 2 but:%d", size);
	} else {
		uint16_t bat_val = data[0] << 8;
		bat_val |= data[1];
		uint16_t msv = bat_val / 1000;
		uint16_t lsv = bat_val % 1000;
		return sprintf(p_msg, "voltage:%u.%03u", msv, lsv);
	}
}

int rx_text(char *p_msg, uint8_t *data, uint8_t size) {
	memcpy(p_msg, data, size);
	return size;
}

void mesh_parse(message_t *msg, char *p_msg) {
	p_msg += sprintf(p_msg, ">");
	p_msg += sprintf(p_msg, "pid:%d;ctrl:0x%02X;src:%d;", msg->pid,
			msg->control, msg->source);
	if (!(MESH_IS_BROADCAST(msg->control))) {
		p_msg += sprintf(p_msg, "dest:%d;", msg->dest);
	}
	if (MESH_IS_ACKNOWLEDGE(msg->control)) {
		p_msg += sprintf(p_msg, "ack:1;");
	}
	switch (msg->pid) {
	case MESH_PID_ALIVE: {
		p_msg += rx_alive(p_msg, msg->payload, msg->payload_length, msg->rssi);
	}
		break;
	case MESH_PID_RESET: {
		p_msg += rx_reset(p_msg, msg->payload, msg->payload_length);
	}
		break;
	case MESH_PID_ENV: {
		p_msg += rx_env(p_msg, msg->payload, msg->payload_length);
	}
		break;
	case MESH_PID_VOC: {
		p_msg += rx_voc(p_msg, msg->payload, msg->payload_length);
	}
		break;
	case MESH_PID_MOTION: {
		p_msg += rx_motion(p_msg, msg->payload, msg->payload_length);
	}
		break;

	case MESH_PID_RANGE: {
		p_msg += rx_range(p_msg, msg->payload, msg->payload_length);
	}
		break;

	case MESH_PID_CO2: {
		p_msg += rx_co2(p_msg, msg->payload, msg->payload_length);
	}
		break;

	case MESH_PID_ALS: {
		p_msg += rx_als(p_msg, msg->payload, msg->payload_length);
	}
		break;

	case MESH_PID_BATTERY: {
		p_msg += rx_battery(p_msg, msg->payload, msg->payload_length);
	}
		break;

	case MESH_PID_TEXT: {
		p_msg += rx_text(p_msg, msg->payload, msg->payload_length);
	}
		break;
	case MESH_PID_EXE: {
		//only responses directed to us are parsed
		//p_msg += cmd_parse_response(p_msg,msg->payload,msg->payload_length);
	}
		break;
	default: {
		if (msg->payload_length > 0) {
			p_msg += sprintf(p_msg, "payload:");
			p_msg += sprint_buf(p_msg, (const char*) msg->payload,
					msg->payload_length);
		}
	}
		break;
	}
	sprintf(p_msg, "<\r\n");
}

void mesh_parse_raw(message_t *msg, char *p_msg) {
	int add;
	p_msg += sprintf(p_msg, ">");
	add = sprintf(p_msg, "rssi:-%d;nodeid:%d;control:0x%02X", msg->rssi,
			msg->source, msg->control);
	p_msg += add;
	if (msg->pid <= 0xEC) {
		add = sprintf(p_msg, ";pid:%s", pid_name[msg->pid]);
		p_msg += add;
	} else {
		add = sprintf(p_msg, ";pid:0x%02X", msg->pid);
		p_msg += add;
	}
	if (msg->payload_length > 0) {
		add = sprintf(p_msg, ";length:%d;data:0x", msg->payload_length);
		p_msg += add;
	}
	for (int i = 0; i < msg->payload_length; i++) {
		add = sprintf(p_msg, "%02X ", msg->payload[i]);
		p_msg += add;
	}
	sprintf(p_msg, "<\r\n");
}

void mesh_parse_bytes(message_t *msg, char *p_msg) {
	int add;
	p_msg += sprintf(p_msg, ">");
	add = sprintf(p_msg, "0x%02X 0x%02X 0x%02X 0x%02X", msg->control, msg->pid,
			msg->source, msg->dest);
	p_msg += add;
	//CRC takes two bytes
	if (msg->payload_length > 0) {
		add = sprintf(p_msg, "; (%d) payload: 0x", msg->payload_length);
		p_msg += add;
	}
	for (int i = 0; i < msg->payload_length; i++) {
		add = sprintf(p_msg, "%02X ", msg->payload[i]);
		p_msg += add;
	}
	sprintf(p_msg, "<\r\n");
}

uint8_t cmd_parse_response(char *text, uint8_t *data, uint8_t size) {
	uint8_t length;
	switch (data[0]) {
	case MESH_cmd_node_id_set: {
		//TODO persistance of parameters
		length = sprintf(text, "cmd:set_node_id;set:%u;get:%u", data[1],
				data[2]);
	}
		break;
	case MESH_cmd_node_id_get: {
		length = sprintf(text, "cmd:get_node_id;node_id:%u", data[1]);
	}
		break;
	case MESH_cmd_rf_chan_set: {
		length = sprintf(text, "cmd:set_channel;set:%u;get:%u", data[1],
				data[2]);
	}
		break;
	case MESH_cmd_rf_chan_get: {
		length = sprintf(text, "cmd:get_channel;channel:%u", data[1]);
	}
		break;
	case MESH_cmd_crc_set: {
		length = sprintf(text, "cmd:set_crc;set:%u;get:%u", data[1], data[2]);
	}
		break;
	case MESH_cmd_crc_get: {
		length = sprintf(text, "cmd:get_crc;crc:%u", data[1]);
	}
		break;
	default: {
		length = sprintf(text, "cmd:0x%02X;resp:unknown", data[0]);
	}
		break;
	}
	return length;
}

/**
 * @brief Executes a command received in a binary format
 *
 * @param data the array starting with <cmd_id> followed by <param0><param1>,...
 * @param size the total size including the first byte of cmd_id
 */
void mesh_execute_cmd(uint8_t *data, uint8_t size, bool is_rf_request,
		uint8_t rf_nodeid) {
	uint8_t resp[32];
	uint8_t resp_len;
	resp[0] = data[0];  //First byte response is always the command id requested
	switch (data[0]) {
	case MESH_cmd_node_id_set: {
		//TODO persistance of parameters
		resp[1] = data[1];          //set request confirmation
		resp[2] = get_this_node_id(); //new set value as read from source after set
		resp_len = 3;
	}
		break;
	case MESH_cmd_node_id_get: {
		resp[1] = get_this_node_id();   //Value as read from source after set
		resp_len = 2;
	}
		break;
	case MESH_cmd_rf_chan_set: {
		mesh_wait_tx();   //in case any action was ongoing
		nrf_esb_stop_rx();
		nrf_esb_set_rf_channel(data[1]);
		nrf_esb_start_rx();
		resp[1] = data[1];          //set channel request confirmation
		resp[2] = mesh_get_channel(); //new set value as read from source after set
		resp_len = 3;
	}
		break;
	case MESH_cmd_rf_chan_get: {
		resp[1] = mesh_get_channel(); //new set value as read from source after set
		resp_len = 2;
	}
		break;
	case MESH_cmd_crc_set: {
		mesh_wait_tx();   //in case any action was ongoing
		nrf_esb_stop_rx();
		mesh_set_crc(data[1]);
		nrf_esb_start_rx();
		resp[1] = data[1];              //set crc request confirmation
		resp[2] = mesh_get_crc();  //new set value as read from source after set
		resp_len = 3;
	}
		break;
	case MESH_cmd_crc_get: {
		resp[1] = mesh_get_crc();  //new set value as read from source after set
		resp_len = 2;
	}
		break;
	default: {
		resp_len = 1;
	}
		break;
	}
	if (is_rf_request) {
		mesh_ucast_data(MESH_PID_EXE, rf_nodeid, resp, resp_len);
	} else {
		char text[128];
		uint8_t length = cmd_parse_response(text, resp, resp_len);
		m_app_cmd_handler(text, length);
	}
}

/**
 * @brief executes the command immediatly
 * Future extension should use a command fifo
 *
 * @param text contains a command line to execute a mesh rf function
 * supported commands :
 * * msg:0x00112233445566... note length not included as will be generated
 * where 0:control , 1:source , 2:dest/payload , 3:payload,...
 * * cmd:0x00112233445566
 * where 0x00 is the command id
 * the rest are the command parameters including : crc_cfg, header_cfg, bitrate,...
 * @param length number of characters in msg
 */
void mesh_text_request(const char *text, uint8_t length) {
	if (strbegins(text, "msg:")) {
		uint8_t data[32];       //TODO define global max cmd size
		uint8_t size;
		if (text2bin(text + 4, length - 4, data, &size)) {
			mesh_tx_raw(data, size);
			char resp[32];
			uint8_t resp_len = sprintf(resp, "sent_msg_len:%d", size);
			m_app_cmd_handler(resp, resp_len);
		}
	} else if (strbegins(text, "cmd:")) {
		uint8_t data[32];       //TODO define global max cmd size
		uint8_t size;
		if (text2bin(text + 4, length - 4, data, &size)) {
			mesh_execute_cmd(data, size, false, 0); //rf_nodeid unused in this case, set as 0
		}
	}
}
