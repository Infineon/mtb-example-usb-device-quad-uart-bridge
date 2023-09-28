/******************************************************************************
* File Name: uart_block.h
*
* Description: This file is the public interface of uart_block.c source file
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

#ifndef UART_BLOCK_H
#define UART_BLOCK_H


#include <stdio.h>
#include <stdint.h>

/* UART Constants */
#define UART_RESET_ALL 0u
#define UART_RESET_TX 1u
#define UART_RESET_RX 2u

#define UART_RTS_FIFO_LVL 4u

/* UART Block Functions */
void uart_block_init(void);
void uart_block_put_data(const uint8_t *data_ptr, uint16_t length, uint8_t port);
uint16_t uart_block_get_count(uint8_t port);
uint16_t uart_block_get_data(uint8_t *data_ptr, uint16_t length, uint8_t port);
void uart_block_update_config(uint32_t baudrate, uint8_t stopbit, uint8_t parity, uint8_t databit, uint8_t port);
void uart_block_reset(uint8_t reset_type, uint8_t port);
uint8_t uart_block_get_status(uint8_t port);

#endif /* UART_BLOCK_H*/
/* [] END OF FILE */
