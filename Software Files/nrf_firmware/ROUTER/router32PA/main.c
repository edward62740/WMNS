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
	bool is_relevant_host = false;
	NRF_LOG_INFO("rf_mesh_handler()");
	nrf_gpio_pin_toggle(PWR_LED);
	if (MESH_IS_BROADCAST(msg->control)) {
		is_relevant_host = true;
	} else if ((msg->dest == get_this_node_id())) {
		if (MESH_IS_RESPONSE(msg->control)) {
			is_relevant_host = true;
		}
		//else it's a request or a message
		else if ((msg->pid == MESH_PID_EXE) && (UICR_is_rf_cmd())) {
			mesh_execute_cmd(msg->payload, msg->payload_length, true,
					msg->source);
		}
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

void app_serial_handler(const char *msg, uint8_t size) {
	uart_rx_size += size;
	//the input (msg) is really the RX DMA pointer location
	//and the output (uart_message) is reall the TX DMA pointer location
	//so have to copy here to avoid overwriting

	mesh_text_request(msg, size);
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
	mesh_tx_alive();
           //returns an incrementing counter
}

int main(void) {
	uint32_t err_code;

	// ------------------------- Start Init -------------------------
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();

	clocks_start();
	nrf_gpio_cfg_output(PWR_LED); // 1
	nrf_gpio_cfg_output(LNA_EN); //en rx
	nrf_gpio_cfg_output(PA_EN); //en tx
	nrf_gpio_cfg_output(RX_LED); // 2
	nrf_gpio_cfg_output(TX_LED); // 3
	ser_init(app_serial_handler);

	//Cannot use non-blocking with buffers from const code memory
	//reset is a status which single event is reset, that status takes the event name
	sprintf(rtc_message, "nodeid:%d;channel:%d;reset:1\r\n", get_this_node_id(),
			mesh_channel());
	ser_send(rtc_message);

	err_code = mesh_init(rf_mesh_handler, mesh_cmd_response);
	APP_ERROR_CHECK(err_code);

	//only allow interrupts to start after init is done
	rtc_config(app_rtc_handler);

	mesh_tx_reset();
	mesh_wait_tx();

	// ------------------------- Start Events -------------------------

	while (true) {
		mesh_consume_rx_messages();
		nrf_delay_ms(1);
		//TODO required delay as the serial_write does not execute with two close consecutive calls

	}
}
/*lint -restore */
