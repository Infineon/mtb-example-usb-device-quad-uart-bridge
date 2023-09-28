/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the USB Device CDC echo Example
 *              for ModusToolbox.
 *
 * Related Document: See README.md
 *
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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "USB.h"
#include "quad_serial.h"
#include "uart_block.h"
#include <stdio.h>

/*******************************************************************************
 * Macros
 ********************************************************************************/
#define BUF_SIZE          128u

/* Communication States */
#define STATE_IDLE        0u
#define STATE_RECEIVE     1u
#define STATE_SEND        2u
#define STATE_SEND_MORE   3u
#define STATE_ZERO_PKT    4u
#define STATE_WAIT        5u

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
uint8_t tx_buffer[CY_NUM_PORTS][BUF_SIZE];
uint8_t rx_buffer[CY_NUM_PORTS][BUF_SIZE];

uint8_t tx_state[CY_NUM_PORTS] = {STATE_IDLE,
                                 STATE_IDLE,
                                 STATE_IDLE,
                                 STATE_IDLE};

uint8_t rx_state[CY_NUM_PORTS] = {STATE_IDLE,
                                 STATE_IDLE,
                                 STATE_IDLE,
                                 STATE_IDLE};

uint16_t tx_count[CY_NUM_PORTS];
uint16_t rx_count[CY_NUM_PORTS];

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * This is the main function for CM4 CPU.
 *
 *  1. It initializes the USB Device block
 *  and enumerates as a CDC device.
 *
 *  2. It receives data from host
 *  and echos it back
 *
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    uint8_t idx;
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "emUSB Device: Quad USB to UART bridge application "
           "****************** \r\n\n");

    uart_block_init();
    quad_serial_init();

    uart_block_put_data((const uint8_t *)"\r\n-----UART 0-----\r\n", 20, 0);
    uart_block_put_data((const uint8_t *)"\r\n-----UART 1-----\r\n", 20, 1);
    uart_block_put_data((const uint8_t *)"\r\n-----UART 2-----\r\n", 20, 2);
    uart_block_put_data((const uint8_t *)"\r\n-----UART 3-----\r\n", 20, 3);

    while (1)
    {
        /* Service USB Quad Serial when device is configured */
        if (USBD_IsConfigured() != 0u)
        {
            /* Check if data was received in any of the ports */
            for (idx = 0; idx < CY_NUM_PORTS; idx++)
            {
                /* Handle Tx State Machine for each Port */
                switch (tx_state[idx])
                {
                case STATE_IDLE:
                    /* Read received data */
                    tx_count[idx] = quad_serial_get_all(tx_buffer[idx], idx);
                    /* Check if data is valid */
                    if ((tx_count[idx] != 0u) && (tx_count[idx] <= CY_RX_EP_BUFFER_SIZE))
                    {
                        tx_state[idx] = STATE_SEND;
                    }
                    else
                    {
                        tx_state[idx] = STATE_IDLE;
                    }
                    break;

                case STATE_SEND:
                    /* Transmit the data to the UART block */
                    uart_block_put_data(tx_buffer[idx], tx_count[idx], idx);
                    tx_state[idx] = STATE_WAIT;
                    break;

                case STATE_WAIT:
                    /* Check if the transmission is completed */
                    if (uart_block_get_status(idx))
                    {
                        tx_state[idx] = STATE_IDLE;
                    }
                    break;

                default:
                    tx_state[idx] = STATE_IDLE;
                    break;
                }

                /* Handle Rx State Machine for each Port */
                switch (rx_state[idx])
                {
                case STATE_IDLE:
                    /* Check for input data from UART */
                    rx_count[idx] = uart_block_get_count(idx);

                    /* If any data is return, move to the next state */
                    if (rx_count[idx] > 0u)
                    {
                        /* Read data from the UART ports */
                        uart_block_get_data(rx_buffer[idx], rx_count[idx], idx);
                        rx_state[idx] = STATE_SEND;
                    }
                    break;

                case STATE_SEND:
                    /* Wait till component is ready to send more data to the PC */
                    if (quad_serial_is_ready(idx) != 0u)
                    {
                        /* Check if multiple packets need to be sent */
                        /* Need to consider the two bytes port status */
                        if (rx_count[idx] < (CY_TX_EP_BUFFER_SIZE - 2))
                        {
                            /* Send data back to PC */
                            quad_serial_put_data(rx_buffer[idx], rx_count[idx], idx);

                            /* No extra packet needed, move to the idle state */
                            rx_state[idx] = STATE_IDLE;
                        }
                        else
                        {
                            /* Send data back to PC */
                            quad_serial_put_data(rx_buffer[idx], (CY_TX_EP_BUFFER_SIZE - 2), idx);

                            /*  If the last sent packet is exactly maximum packet size,
                             *  it shall be followed by a zero-length packet to assure the
                             *  end of segment is properly identified by the terminal.
                             */
                            if (rx_count[idx] == (CY_TX_EP_BUFFER_SIZE - 2))
                            {
                                rx_state[idx] = STATE_ZERO_PKT;
                            }
                            else /* If not, an extra packet is needed to be sent */
                            {
                                rx_state[idx] = STATE_SEND_MORE;
                            }
                        }
                    }
                    break;

                case STATE_SEND_MORE:
                    /* Wait till component is ready to send more data to the PC */
                    if (quad_serial_is_ready(idx) != 0u)
                    {
                        /* Send the rest of the packet */
                        quad_serial_put_data(&rx_buffer[idx][CY_TX_EP_BUFFER_SIZE - 2],
                                             rx_count[idx] - (CY_TX_EP_BUFFER_SIZE - 2), idx);

                        rx_state[idx] = STATE_IDLE;
                    }
                    break;

                case STATE_ZERO_PKT:
                    /* Check if ready to send more data to the PC */
                    if (quad_serial_is_ready(idx) != 0u)
                    {
                        /* Send zero-length packet to PC */
                        quad_serial_put_data(NULL, 0u, idx);

                        rx_state[idx] = STATE_IDLE;
                    }
                    break;

                default:
                    rx_state[idx] = STATE_IDLE;
                    break;
                }
            }
        }
    }
}
/* [] END OF FILE */
