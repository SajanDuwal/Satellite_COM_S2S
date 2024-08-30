/*
 * obc_interface.c
 *
 *  Created on: Aug 29, 2024
 *      Author: sajanduwal
 */

#include "obc_interface.h"

#define ACK_HEAD	(0x53)
#define ACK_TAIL	(0x7E)
#define ACK_LENGTH	(7)

extern uint8_t OBC_HANDSHAKE_FLAG;
uint8_t MainCMDHs[ACK_LENGTH];

void WAIT_FOR_HANDSHAKE() {

	memset(MainCMDHs, '\0', ACK_LENGTH);
	OBC_HANDSHAKE_FLAG = 0;
	if (HAL_UART_Receive(&huart2, MainCMDHs, ACK_LENGTH, 7000) == HAL_OK
			|| HAL_UART_Receive(&hlpuart1, MainCMDHs, ACK_LENGTH, 7000)
					== HAL_OK) {
		myDebug("--> Handshake command received from OBC: 0x%x\r\n");
		for (int i = 0; i < (ACK_LENGTH); i++) {
			myDebug("%02x ", MainCMDHs[i]);
		}
		myDebug("\n");

		uint8_t header = 0x00;

		if (MainCMDHs[0] == header) {

			for (int loop1 = 0; loop1 < sizeof(MainCMDHs); loop1++) {
				MainCMDHs[loop1] = MainCMDHs[loop1 + 1];
			}
		}

		if (MainCMDHs[0] == ACK_HEAD && MainCMDHs[5] == ACK_TAIL) {
			myDebug("--> Command Acknowledged successful!\n");
			if (HAL_UART_Transmit(&huart2, MainCMDHs, ACK_LENGTH, 2000)
					== HAL_OK
					|| HAL_UART_Transmit(&hlpuart1, MainCMDHs, ACK_LENGTH, 2000)
							== HAL_OK) {
				myDebug("--> Handshake ACK, re-transmit to OBC: \n");
				for (int i = 0; i < (ACK_LENGTH); i++) {
					myDebug("%02x ", MainCMDHs[i]);
				}
				myDebug("\n");
				OBC_HANDSHAKE_FLAG = 1;
				memset(MainCMDHs, '\0', ACK_LENGTH);
			}
		} else {
			myDebug("*** Unknown Handshake command received!\n");
			if (HAL_UART_Transmit(&huart2, MainCMDHs, ACK_LENGTH, 2000)
					== HAL_OK
					|| HAL_UART_Transmit(&hlpuart1, MainCMDHs, ACK_LENGTH, 7000)
							== HAL_OK) {
				myDebug("--> Unknown Handshake ACK, re-transmit to OBC.\n");
				for (int i = 0; i < (ACK_LENGTH); i++) {
					myDebug("%02x ", MainCMDHs[i]);
				}
				myDebug("\n");
				memset(MainCMDHs, '\0', ACK_LENGTH);
				OBC_HANDSHAKE_FLAG = 0;
				WAIT_FOR_HANDSHAKE();
			}
		}
	} else {
		OBC_HANDSHAKE_FLAG = 0;
		myDebug("*** Handshake Command receive failed, try again!\n");
		memset(MainCMDHs, '\0', ACK_LENGTH);
		WAIT_FOR_HANDSHAKE();
	}
}
