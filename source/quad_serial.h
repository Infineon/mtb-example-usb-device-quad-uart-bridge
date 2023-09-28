/******************************************************************************
* File Name: quad_serial.h
*
* Description: This file is the public interface of quad_serial.c source file
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef QUAD_SERIAL_H
#define QUAD_SERIAL_H

#include <stdio.h>
#include <stdint.h>
#define CY_NUM_PORTS (4u)

#define CY_TX_EP_BUFFER_SIZE (64u)
#define CY_RX_EP_BUFFER_SIZE (64u)

#define USB_CONFIG_DELAY (50U) /* In milliseconds */

/* Quad Serial Functions */
void quad_serial_init(void);
void quad_serial_put_data(const uint8_t *data_ptr, uint16_t length, uint8_t port);
void quad_serial_put_string(const char string[], uint8_t port);
void quad_serial_put_char(char tx_data_byte, uint8_t port);
void quad_serial_put_crlf(uint8_t port);
uint8_t quad_serial_is_ready(uint8_t port);
uint8_t quad_serial_data_is_ready(uint8_t port);
uint16_t quad_serial_get_data(uint8_t *data_ptr, uint16_t length, uint8_t port);
uint16_t quad_serial_get_all(uint8_t *data_ptr, uint8_t port);
uint8_t quad_serial_get_char(uint8_t port);

#endif /* QUAD_SERIAL_H */
/* [] END OF FILE */
