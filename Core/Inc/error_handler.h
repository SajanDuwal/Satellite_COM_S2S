/*
 * error_handler.h
 *
 *  Created on: Aug 29, 2024
 *      Author: sajanduwal
 */

#ifndef INC_ERROR_HANDLER_H_
#define INC_ERROR_HANDLER_H_

#include "main.h"
#include "com_debug.h"

uint16_t calculateCRC_CCITT_AX25(const uint8_t *data, size_t length);

uint16_t calc_CRC(const uint8_t *data, size_t length);

int bit_stuffing(uint8_t *data, uint8_t *output_data, int length);

int bit_destuffing(uint8_t *data, uint8_t *output_data, int length);

int check_packet_type(uint8_t *OBC_UART);

#endif /* INC_ERROR_HANDLER_H_ */
