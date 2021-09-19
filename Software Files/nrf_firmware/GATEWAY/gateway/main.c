#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_util.h"
//for the log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "uicr_user_defines.h"
//drivers
//apps
#include "clocks.h"
#include "utils.h"
#include "mesh.h"
#include "app_ser.h"
#include "nrf_mtx.h"

char rtc_message[64];
char uart_message[64];
char rf_message[128];
uint32_t uart_rx_size = 0;

/**
 * @brief callback from the RF Mesh stack on valid packet received for this node
 *
 * @param msg message structure with packet parameters and payload
 */
void rf_mesh_handler(message_t *msg) {

	nrf_gpio_pin_toggle(MESH_LED);
	bool is_relevant_host = false;
	NRF_LOG_INFO("rf_mesh_handler()");
	if (MESH_IS_BROADCAST(msg->control)) {
		is_relevant_host = true;
	} else if ((msg->dest == get_this_node_id())) {
		if (MESH_IS_RESPONSE(msg->control)) {
			is_relevant_host = true;
		}
		//else it's a request or a message
	}
	if (is_relevant_host) {
		//Pure routers should not waste time sending messages over uart
		if (UICR_is_rf2uart()) {
			mesh_parse(msg, rf_message);
			ser_send(rf_message); // collision
		}
	}
}

/**
 * @brief called only with a full line message coming from UART
 * ending with '\r', '\n' or '0'
 * @param msg contains a pointer to the DMA buffer, so do not keep it after the call
 * @param size safe managemnt with known size, does not include the ending char '\r' or other
 */
#define UART_MIRROR
void app_serial_handler(const char *msg, uint8_t size) {
	uart_rx_size += size;
	//the input (msg) is really the RX DMA pointer location
	//and the output (uart_message) is reall the TX DMA pointer location
	//so have to copy here to avoid overwriting
#ifdef UART_MIRROR
	memcpy(uart_message, msg, size);
	sprintf(uart_message + size, "\r\n"); //Add line ending and NULL terminate it with sprintf
	ser_send(uart_message);
#endif

	mesh_text_request(msg,size);
}

/**
 * @brief Callback from after completion of the cmd request
 * Note this is a call back as some undefined latency and events might happen
 * before the response is ready, thus the requests cannot always return immidiatly
 *
 * @param text
 * @param length
 */
void mesh_cmd_response(const char *text, uint8_t length) {
	memcpy(uart_message, text, length);
	sprintf(uart_message + length, "\r\n"); //Add line ending and NULL terminate it with sprintf
	ser_send(uart_message);    // collision
}

/**
 * @brief application rtc event which is a configurable period delay
 * through the uicr config "sleep" in the nodes databse
 *
 */
void app_rtc_handler() {
	uint32_t alive_count = mesh_tx_alive();    //returns an incrementing counter
	nrf_gpio_pin_toggle(ACT_LED);
	NRF_LOG_INFO("id:%d:alive:%lu", get_this_node_id(), alive_count);
	UNUSED_VARIABLE(alive_count);
}

int main(void) {
	uint32_t err_code;

	// ------------------------- Start Init -------------------------
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();

	clocks_start();

	ser_init(app_serial_handler);

	//Cannot use non-blocking with buffers from const code memory
	//reset is a status which single event is reset, that status takes the event name
	sprintf(rtc_message, "nodeid:%d;channel:%d;reset:1|\r\n",
			get_this_node_id(), mesh_channel());
	ser_send(rtc_message);

	err_code = mesh_init(rf_mesh_handler, mesh_cmd_response);
	APP_ERROR_CHECK(err_code);

	//only allow interrupts to start after init is done
	//rtc_config(app_rtc_handler);

	mesh_tx_reset();

	// ------------------------- Start Events -------------------------
	nrf_gpio_cfg_output(MESH_LED);
	nrf_gpio_cfg_output(ACT_LED);

	while (true) {
		mesh_consume_rx_messages();
		nrf_delay_us(10);
		//TODO required delay as the serial_write does not execute with two close consecutive calls
	}
}
/*lint -restore */
