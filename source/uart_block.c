/******************************************************************************
* File Name: uart_block.c
*
* Description: This file contains source code that controls UART Block.
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

#include "uart_block.h"
#include "quad_serial.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "USB.h"
#include <stdio.h>

/*******************************************************************************
 * Macros
 *******************************************************************************/

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
cy_stc_scb_uart_context_t uart0_context;
cy_stc_scb_uart_context_t uart1_context;
cy_stc_scb_uart_context_t uart2_context;
cy_stc_scb_uart_context_t uart3_context;

volatile uint8_t uart0_tx_dma_complete_status = 0u;
volatile uint8_t uart1_tx_dma_complete_status = 0u;
volatile uint8_t uart2_tx_dma_complete_status = 0u;
volatile uint8_t uart3_tx_dma_complete_status = 0u;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static void uart0_tx_dma_complete(void);
static void uart1_tx_dma_complete(void);
static void uart2_tx_dma_complete(void);
static void uart3_tx_dma_complete(void);

static void handle_error(void);

/*******************************************************************************
 * Function Name: uart_block_init
 ********************************************************************************
 *
 * Summary:
 *  This function initialize the UART blocks.
 *
 * Parameters:
 *  None.
 *
 * Return:
 *  None.
 *
 * Global variables:
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
void uart_block_init(void)
{
    cy_en_scb_uart_status_t init_status;

    /*********************************************************************
     * UART 0
     ********************************************************************/
    cy_stc_sysint_t UART0_TX_DMA_INT_cfg =
        {
            .intrSrc = (IRQn_Type)UART0_TxDma_IRQ,
            .intrPriority = 3u,
        };

    cy_en_dma_status_t dma_init_status;

    /* Init descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init(&UART0_TxDma_Descriptor_0, &UART0_TxDma_Descriptor_0_config);
    if (dma_init_status != CY_DMA_SUCCESS)
    {
        handle_error();
    }

    /* Set source and destination for descriptor 1 */
    Cy_DMA_Descriptor_SetDstAddress(&UART0_TxDma_Descriptor_0, (uint32_t *)&UART_0_HW->TX_FIFO_WR);

    /* Set next descriptor to NULL to stop the chain execution after descriptor 1
     *  is completed.
     */
    Cy_DMA_Descriptor_SetNextDescriptor(&UART0_TxDma_Descriptor_0, &UART0_TxDma_Descriptor_0);

    /* Initialize and enable the interrupt from TxDma */
    Cy_SysInt_Init(&UART0_TX_DMA_INT_cfg, &uart0_tx_dma_complete);
    NVIC_EnableIRQ(UART0_TX_DMA_INT_cfg.intrSrc);

    /* Enable DMA interrupt source */
    Cy_DMA_Channel_SetInterruptMask(UART0_TxDma_HW, UART0_TxDma_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable Data Write block but keep channel disabled to not trigger
     *  descriptor execution because TX FIFO is empty and SCB keeps active level
     *  for DMA.
     */
    Cy_DMA_Enable(UART0_TxDma_HW);

    /* Start UART_0 operation */
    init_status = Cy_SCB_UART_Init(UART_0_HW, &UART_0_config, &uart0_context);
    if (init_status != CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
    Cy_SCB_UART_Enable(UART_0_HW);

    /*********************************************************************
     * UART 1
     ********************************************************************/
    cy_stc_sysint_t UART1_TX_DMA_INT_cfg =
        {
            .intrSrc = (IRQn_Type)UART1_TxDma_IRQ,
            .intrPriority = 3u,
        };

    /* Init descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init(&UART1_TxDma_Descriptor_0, &UART1_TxDma_Descriptor_0_config);
    if (dma_init_status != CY_DMA_SUCCESS)
    {
        handle_error();
    }

    /* Set source and destination for descriptor 1 */
    Cy_DMA_Descriptor_SetDstAddress(&UART1_TxDma_Descriptor_0, (uint32_t *)&UART_1_HW->TX_FIFO_WR);

    /* Set next descriptor to NULL to stop the chain execution after descriptor 1
     *  is completed.
     */
    Cy_DMA_Descriptor_SetNextDescriptor(&UART1_TxDma_Descriptor_0, &UART1_TxDma_Descriptor_0);

    /* Initialize and enable the interrupt from TxDma */
    Cy_SysInt_Init(&UART1_TX_DMA_INT_cfg, &uart1_tx_dma_complete);
    NVIC_EnableIRQ(UART1_TX_DMA_INT_cfg.intrSrc);

    /* Enable DMA interrupt source */
    Cy_DMA_Channel_SetInterruptMask(UART1_TxDma_HW, UART1_TxDma_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable Data Write block but keep channel disabled to not trigger
     *  descriptor execution because TX FIFO is empty and SCB keeps active level
     *  for DMA.
     */
    Cy_DMA_Enable(UART1_TxDma_HW);

    /* Start UART_1 operation */
    init_status = Cy_SCB_UART_Init(UART_1_HW, &UART_1_config, &uart1_context);
    if (init_status != CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
    Cy_SCB_UART_Enable(UART_1_HW);

    /*********************************************************************
     * UART 2
     ********************************************************************/
    cy_stc_sysint_t UART2_TX_DMA_INT_cfg =
        {
            .intrSrc = (IRQn_Type)UART2_TxDma_IRQ,
            .intrPriority = 3u,
        };

    /* Init descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init(&UART2_TxDma_Descriptor_0, &UART2_TxDma_Descriptor_0_config);
    if (dma_init_status != CY_DMA_SUCCESS)
    {
        handle_error();
    }

    /* Set source and destination for descriptor 1 */
    Cy_DMA_Descriptor_SetDstAddress(&UART2_TxDma_Descriptor_0, (uint32_t *)&UART_2_HW->TX_FIFO_WR);

    /* Set next descriptor to NULL to stop the chain execution after descriptor 1
     *  is completed.
     */
    Cy_DMA_Descriptor_SetNextDescriptor(&UART2_TxDma_Descriptor_0, &UART2_TxDma_Descriptor_0);

    /* Initialize and enable the interrupt from TxDma */
    Cy_SysInt_Init(&UART2_TX_DMA_INT_cfg, &uart2_tx_dma_complete);
    NVIC_EnableIRQ(UART2_TX_DMA_INT_cfg.intrSrc);

    /* Enable DMA interrupt source */
    Cy_DMA_Channel_SetInterruptMask(UART2_TxDma_HW, UART2_TxDma_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable Data Write block but keep channel disabled to not trigger
     *  descriptor execution because TX FIFO is empty and SCB keeps active level
     *  for DMA.
     */
    Cy_DMA_Enable(UART2_TxDma_HW);

    /* Start UART_1 operation */
    init_status = Cy_SCB_UART_Init(UART_2_HW, &UART_2_config, &uart2_context);
    if (init_status != CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
    Cy_SCB_UART_Enable(UART_2_HW);

    /*********************************************************************
     * UART 3
     ********************************************************************/
    cy_stc_sysint_t UART3_TX_DMA_INT_cfg =
        {
            .intrSrc = (IRQn_Type)UART3_TxDma_IRQ,
            .intrPriority = 3u,
        };

    /* Init descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init(&UART3_TxDma_Descriptor_0, &UART3_TxDma_Descriptor_0_config);
    if (dma_init_status != CY_DMA_SUCCESS)
    {
        handle_error();
    }

    /* Set source and destination for descriptor 1 */
    Cy_DMA_Descriptor_SetDstAddress(&UART3_TxDma_Descriptor_0, (uint32_t *)&UART_3_HW->TX_FIFO_WR);

    /* Set next descriptor to NULL to stop the chain execution after descriptor 1
     *  is completed.
     */
    Cy_DMA_Descriptor_SetNextDescriptor(&UART3_TxDma_Descriptor_0, &UART3_TxDma_Descriptor_0);

    /* Initialize and enable the interrupt from TxDma */
    Cy_SysInt_Init(&UART3_TX_DMA_INT_cfg, &uart3_tx_dma_complete);
    NVIC_EnableIRQ(UART3_TX_DMA_INT_cfg.intrSrc);

    /* Enable DMA interrupt source */
    Cy_DMA_Channel_SetInterruptMask(UART3_TxDma_HW, UART3_TxDma_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable Data Write block but keep channel disabled to not trigger
     *  descriptor execution because TX FIFO is empty and SCB keeps active level
     *  for DMA.
     */
    Cy_DMA_Enable(UART3_TxDma_HW);

    /* Start UART_1 operation */
    init_status = Cy_SCB_UART_Init(UART_3_HW, &UART_3_config, &uart3_context);
    if (init_status != CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
    Cy_SCB_UART_Enable(UART_3_HW);
}

/*******************************************************************************
 * Function Name: tx_dma_complete
 ********************************************************************************
 *
 * Summary:
 *  Handles Tx Dma descriptor completion interrupt source: only used for
 *  indication.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
static void uart0_tx_dma_complete(void)
{
    /* Check interrupt cause to capture errors.
     *  Note that next descriptor is NULL to stop descriptor execution */
    if ((CY_DMA_INTR_CAUSE_COMPLETION != Cy_DMA_Channel_GetStatus(UART0_TxDma_HW, UART0_TxDma_CHANNEL)) &&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(UART0_TxDma_HW, UART0_TxDma_CHANNEL)))
    {
        /* DMA error occurred while TX operations */
    }
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(UART0_TxDma_HW, UART0_TxDma_CHANNEL))
    {
        uart0_tx_dma_complete_status = true;
    }
    Cy_DMA_Channel_ClearInterrupt(UART0_TxDma_HW, UART0_TxDma_CHANNEL);
}

static void uart1_tx_dma_complete(void)
{
    /* Check interrupt cause to capture errors.
     *  Note that next descriptor is NULL to stop descriptor execution */
    if ((CY_DMA_INTR_CAUSE_COMPLETION != Cy_DMA_Channel_GetStatus(UART1_TxDma_HW, UART1_TxDma_CHANNEL)) &&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(UART1_TxDma_HW, UART1_TxDma_CHANNEL)))
    {
        /* DMA error occurred while TX operations */
    }
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(UART1_TxDma_HW, UART1_TxDma_CHANNEL))
    {
        uart1_tx_dma_complete_status = true;
    }
    Cy_DMA_Channel_ClearInterrupt(UART1_TxDma_HW, UART1_TxDma_CHANNEL);
}

static void uart2_tx_dma_complete(void)
{
    /* Check interrupt cause to capture errors.
     *  Note that next descriptor is NULL to stop descriptor execution */
    if ((CY_DMA_INTR_CAUSE_COMPLETION != Cy_DMA_Channel_GetStatus(UART2_TxDma_HW, UART2_TxDma_CHANNEL)) &&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(UART2_TxDma_HW, UART2_TxDma_CHANNEL)))
    {
        /* DMA error occurred while TX operations */
    }
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(UART2_TxDma_HW, UART2_TxDma_CHANNEL))
    {
        uart2_tx_dma_complete_status = true;
    }
    Cy_DMA_Channel_ClearInterrupt(UART2_TxDma_HW, UART2_TxDma_CHANNEL);
}

static void uart3_tx_dma_complete(void)
{
    /* Check interrupt cause to capture errors.
     *  Note that next descriptor is NULL to stop descriptor execution */
    if ((CY_DMA_INTR_CAUSE_COMPLETION != Cy_DMA_Channel_GetStatus(UART3_TxDma_HW, UART3_TxDma_CHANNEL)) &&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(UART3_TxDma_HW, UART3_TxDma_CHANNEL)))
    {
        /* DMA error occurred while TX operations */
    }
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(UART3_TxDma_HW, UART3_TxDma_CHANNEL))
    {
        uart3_tx_dma_complete_status = true;
    }
    Cy_DMA_Channel_ClearInterrupt(UART3_TxDma_HW, UART3_TxDma_CHANNEL);
}

/*******************************************************************************
 * Function Name: uart_block_put_data
 ********************************************************************************
 *
 * Summary:
 *  Places an array of data into the UART block transmit buffer to be sent.
 *  This function is blocking and waits until there is a space available to put
 *  all the requested data in the transmit buffer. The array size can be greater
 *  than transmit buffer size.
 *
 * Parameters:
 *  data_ptr: pointer to the buffer containing data to be sent.
 *  length: Specifies the number of bytes to send from the data_ptr
 *          buffer. Maximum length will be limited by the maximum packet
 *          size for the endpoint. Data will be lost if length is greater than Max
 *          Packet Size.
 *  port: channel to be used
 *
 * Return:
 *  None.
 *
 * Global variables:
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
void uart_block_put_data(const uint8_t *data_ptr, uint16_t length, uint8_t port)
{
    /* Handle up to 4 UART blocks */
    switch (port)
    {
    case 0:
        uart0_tx_dma_complete_status = 0u;

        Cy_DMA_Descriptor_SetSrcAddress(&UART0_TxDma_Descriptor_0, (uint32_t *)data_ptr);
        Cy_DMA_Descriptor_SetXloopDataCount(&UART0_TxDma_Descriptor_0, length);
        Cy_DMA_Channel_Init(UART0_TxDma_HW, UART0_TxDma_CHANNEL, &UART0_TxDma_channelConfig);
        Cy_DMA_Channel_Enable(UART0_TxDma_HW, UART0_TxDma_CHANNEL);
        break;

    case 1:
        uart1_tx_dma_complete_status = 0u;

        Cy_DMA_Descriptor_SetSrcAddress(&UART1_TxDma_Descriptor_0, (uint32_t *)data_ptr);
        Cy_DMA_Descriptor_SetXloopDataCount(&UART1_TxDma_Descriptor_0, length);
        Cy_DMA_Channel_Init(UART1_TxDma_HW, UART1_TxDma_CHANNEL, &UART1_TxDma_channelConfig);
        Cy_DMA_Channel_Enable(UART1_TxDma_HW, UART1_TxDma_CHANNEL);
        break;

    case 2:
        uart2_tx_dma_complete_status = 0u;

        Cy_DMA_Descriptor_SetSrcAddress(&UART2_TxDma_Descriptor_0, (uint32_t *)data_ptr);
        Cy_DMA_Descriptor_SetXloopDataCount(&UART2_TxDma_Descriptor_0, length);
        Cy_DMA_Channel_Init(UART2_TxDma_HW, UART2_TxDma_CHANNEL, &UART2_TxDma_channelConfig);
        Cy_DMA_Channel_Enable(UART2_TxDma_HW, UART2_TxDma_CHANNEL);
        break;

    case 3:
        uart3_tx_dma_complete_status = 0u;

        Cy_DMA_Descriptor_SetSrcAddress(&UART3_TxDma_Descriptor_0, (uint32_t *)data_ptr);
        Cy_DMA_Descriptor_SetXloopDataCount(&UART3_TxDma_Descriptor_0, length);
        Cy_DMA_Channel_Init(UART3_TxDma_HW, UART3_TxDma_CHANNEL, &UART3_TxDma_channelConfig);
        Cy_DMA_Channel_Enable(UART3_TxDma_HW, UART3_TxDma_CHANNEL);
        break;
    }
}

/*******************************************************************************
 * Function Name: uart_block_get_count
 ********************************************************************************
 *
 * Summary:
 *  This function returns the number of bytes that were received from the UART
 *  block. The returned length value should be passed to uart_block_get_data()
 *  as a parameter to read all received data. If all of the received data is not
 *  read at one time by the uart_block_get_data() API, the unread data will
 *  be lost.
 *
 * Parameters:
 *  None.
 *
 * Return:
 *  Returns the number of received bytes. The maximum amount of received data at
 *  a time is limited by the size of the UART block.
 *
 * Global variables:
 *
 *******************************************************************************/
uint16_t uart_block_get_count(uint8_t port)
{
    uint16_t count = 0;

    switch (port)
    {
    case 0:
        count = Cy_SCB_UART_GetNumInRxFifo(UART_0_HW);
        break;
    case 1:
        count = Cy_SCB_UART_GetNumInRxFifo(UART_1_HW);
        break;
    case 2:
        count = Cy_SCB_UART_GetNumInRxFifo(UART_2_HW);
        break;
    case 3:
        count = Cy_SCB_UART_GetNumInRxFifo(UART_3_HW);
        break;
    }

    return count;
}

/*******************************************************************************
 * Function Name: uart_block_get_data
 ********************************************************************************
 *
 * Summary:
 *  This function gets a specified number of bytes from the input buffer and
 *  places them in a data array specified by the passed pointer.
 *  If all received data will not be read at
 *  once, the unread data will be lost. The uart_block_get_data() API should
 *  be called to get the number of bytes that were received.
 *
 * Parameters:
 *  data_ptr: Pointer to the data array where data will be placed.
 *  length: Number of bytes to read into the data array from the RX buffer.
 *          Maximum length is limited by the the number of received bytes.
 *
 * Return:
 *  Number of bytes received.
 *
 * Global variables:
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
uint16_t uart_block_get_data(uint8_t *data_ptr, uint16_t length, uint8_t port)
{
    uint16_t count = 0;

    switch (port)
    {
    case 0:
        Cy_SCB_UART_GetArray(UART_0_HW, data_ptr, length);
        break;

    case 1:
        Cy_SCB_UART_GetArray(UART_1_HW, data_ptr, length);
        break;

    case 2:
        Cy_SCB_UART_GetArray(UART_2_HW, data_ptr, length);
        break;

    case 3:
        Cy_SCB_UART_GetArray(UART_3_HW, data_ptr, length);
        break;
    }

    return count;
}

/*******************************************************************************
 * Function Name: CyUartBlock_UpdateBaudRate
 ********************************************************************************
 *
 * Summary:
 *  This function updates the baudrate, stopbit, parity and databit width
 *  for the given UART block
 *
 * Parameters:
 *  baudrate: desired baud rate
 *  stopbit: 0 - 1 stop bit
 *           1 - 1.5 stop bits
 *           2 - 2 stop bits
 *  parity:  0 - None
 *           1 - Odd
 *           2 - Even
 *  databit: witdh of the data bits
 *  port: channel to be used
 *
 * Return:
 *
 * Global variables:
 *
 *******************************************************************************/
void uart_block_update_config(uint32_t baudrate, uint8_t stopbit, uint8_t parity, uint8_t databit, uint8_t port)
{
    uint32_t divider;

    divider = (115200UL * 104UL) / baudrate;

    /* Stop, set configuration, start UART block */
    switch (port)
    {
    case 0:
        /* Disable UART operation before changing the setting*/
        Cy_SCB_UART_Disable(UART_0_HW, &uart0_context);

        Cy_SCB_UART_DeInit(UART_0_HW);

        /* Set Baud Rate */
        Cy_SysClk_PeriphDisableDivider(UART0_Clk_HW, UART0_Clk_NUM);
        Cy_SysClk_PeriphSetDivider(UART0_Clk_HW, UART0_Clk_NUM, divider - 1);
        Cy_SysClk_PeriphEnableDivider(UART0_Clk_HW, UART0_Clk_NUM);

        /* Initialize SCB for UART operation */
        (void)Cy_SCB_UART_Init(UART_0_HW, &UART_0_config, &uart0_context);

        /* Set stop bits value for UART */
        switch (stopbit)
        {
        case 0:
            Cy_SCB_UART_SetStopBits(UART_0_HW, CY_SCB_UART_STOP_BITS_1);
            break;
        case 1:
            Cy_SCB_UART_SetStopBits(UART_0_HW, CY_SCB_UART_STOP_BITS_1_5);
            break;
        case 2:
            Cy_SCB_UART_SetStopBits(UART_0_HW, CY_SCB_UART_STOP_BITS_2);
            break;
        default:
            break;
        }

        /* Set parity value for UART */
        switch (parity)
        {
        case 0:
            Cy_SCB_UART_SetParity(UART_0_HW, CY_SCB_UART_PARITY_NONE);
            break;
        case 1:
            Cy_SCB_UART_SetParity(UART_0_HW, CY_SCB_UART_PARITY_ODD);
            break;
        case 2:
            Cy_SCB_UART_SetParity(UART_0_HW, CY_SCB_UART_PARITY_EVEN);
            break;
        default:
            break;
        }

        /* Set parity value for UART */
        Cy_SCB_UART_SetDataWidth(UART_0_HW, (uint32_t)databit);

        /* Enable UART to operate */
        Cy_SCB_UART_Enable(UART_0_HW);
        break;

    case 1:
        /* Disable UART operation before changing the setting*/
        Cy_SCB_UART_Disable(UART_1_HW, &uart1_context);

        Cy_SCB_UART_DeInit(UART_1_HW);

        /* Set Baud Rate */
        Cy_SysClk_PeriphDisableDivider(UART1_Clk_HW, UART1_Clk_NUM);
        Cy_SysClk_PeriphSetDivider(UART1_Clk_HW, UART1_Clk_NUM, divider - 1);
        Cy_SysClk_PeriphEnableDivider(UART1_Clk_HW, UART1_Clk_NUM);

        /* Initialize SCB for UART operation */
        (void)Cy_SCB_UART_Init(UART_1_HW, &UART_1_config, &uart1_context);

        /* Set stop bits value for UART */
        switch (stopbit)
        {
        case 0:
            Cy_SCB_UART_SetStopBits(UART_1_HW, CY_SCB_UART_STOP_BITS_1);
            break;
        case 1:
            Cy_SCB_UART_SetStopBits(UART_1_HW, CY_SCB_UART_STOP_BITS_1_5);
            break;
        case 2:
            Cy_SCB_UART_SetStopBits(UART_1_HW, CY_SCB_UART_STOP_BITS_2);
            break;
        default:
            break;
        }

        /* Set parity value for UART */
        switch (parity)
        {
        case 0:
            Cy_SCB_UART_SetParity(UART_1_HW, CY_SCB_UART_PARITY_NONE);
            break;
        case 1:
            Cy_SCB_UART_SetParity(UART_1_HW, CY_SCB_UART_PARITY_ODD);
            break;
        case 2:
            Cy_SCB_UART_SetParity(UART_1_HW, CY_SCB_UART_PARITY_EVEN);
            break;
        default:
            break;
        }

        /* Set parity value for UART */
        Cy_SCB_UART_SetDataWidth(UART_1_HW, (uint32_t)databit);

        /* Enable UART to operate */
        Cy_SCB_UART_Enable(UART_1_HW);
        break;

    case 2:
        /* Disable UART operation before changing the setting*/
        Cy_SCB_UART_Disable(UART_2_HW, &uart2_context);

        Cy_SCB_UART_DeInit(UART_2_HW);

        /* Set Baud Rate */
        Cy_SysClk_PeriphDisableDivider(UART2_Clk_HW, UART2_Clk_NUM);
        Cy_SysClk_PeriphSetDivider(UART2_Clk_HW, UART2_Clk_NUM, divider - 1);
        Cy_SysClk_PeriphEnableDivider(UART2_Clk_HW, UART2_Clk_NUM);

        /* Initialize SCB for UART operation */
        (void)Cy_SCB_UART_Init(UART_2_HW, &UART_2_config, &uart2_context);

        /* Set stop bits value for UART */
        switch (stopbit)
        {
        case 0:
            Cy_SCB_UART_SetStopBits(UART_2_HW, CY_SCB_UART_STOP_BITS_1);
            break;
        case 1:
            Cy_SCB_UART_SetStopBits(UART_2_HW, CY_SCB_UART_STOP_BITS_1_5);
            break;
        case 2:
            Cy_SCB_UART_SetStopBits(UART_2_HW, CY_SCB_UART_STOP_BITS_2);
            break;
        default:
            break;
        }

        /* Set parity value for UART */
        switch (parity)
        {
        case 0:
            Cy_SCB_UART_SetParity(UART_2_HW, CY_SCB_UART_PARITY_NONE);
            break;
        case 1:
            Cy_SCB_UART_SetParity(UART_2_HW, CY_SCB_UART_PARITY_ODD);
            break;
        case 2:
            Cy_SCB_UART_SetParity(UART_2_HW, CY_SCB_UART_PARITY_EVEN);
            break;
        default:
            break;
        }

        /* Set parity value for UART */
        Cy_SCB_UART_SetDataWidth(UART_2_HW, (uint32_t)databit);

        /* Enable UART to operate */
        Cy_SCB_UART_Enable(UART_2_HW);
        break;

    case 3:
        /* Disable UART operation before changing the setting*/
        Cy_SCB_UART_Disable(UART_3_HW, &uart3_context);

        Cy_SCB_UART_DeInit(UART_3_HW);

        /* Set Baud Rate */
        Cy_SysClk_PeriphDisableDivider(UART3_Clk_HW, UART3_Clk_NUM);
        Cy_SysClk_PeriphSetDivider(UART3_Clk_HW, UART3_Clk_NUM, divider - 1);
        Cy_SysClk_PeriphEnableDivider(UART3_Clk_HW, UART3_Clk_NUM);

        /* Initialize SCB for UART operation */
        (void)Cy_SCB_UART_Init(UART_3_HW, &UART_3_config, &uart3_context);

        /* Set stop bits value for UART */
        switch (stopbit)
        {
        case 0:
            Cy_SCB_UART_SetStopBits(UART_3_HW, CY_SCB_UART_STOP_BITS_1);
            break;
        case 1:
            Cy_SCB_UART_SetStopBits(UART_3_HW, CY_SCB_UART_STOP_BITS_1_5);
            break;
        case 2:
            Cy_SCB_UART_SetStopBits(UART_3_HW, CY_SCB_UART_STOP_BITS_2);
            break;
        default:
            break;
        }

        /* Set parity value for UART */
        switch (parity)
        {
        case 0:
            Cy_SCB_UART_SetParity(UART_3_HW, CY_SCB_UART_PARITY_NONE);
            break;
        case 1:
            Cy_SCB_UART_SetParity(UART_3_HW, CY_SCB_UART_PARITY_ODD);
            break;
        case 2:
            Cy_SCB_UART_SetParity(UART_3_HW, CY_SCB_UART_PARITY_EVEN);
            break;
        default:
            break;
        }

        /* Set parity value for UART */
        Cy_SCB_UART_SetDataWidth(UART_3_HW, (uint32_t)databit);

        /* Enable UART to operate */
        Cy_SCB_UART_Enable(UART_3_HW);
        break;
    }
}

/*******************************************************************************
 * Function Name: uart_block_reset
 ********************************************************************************
 *
 * Summary:
 *  This function resets the UART block
 *
 * Parameters:
 *  reset_type:
 *  port: channel to be used
 *
 * Return:
 *
 * Global variables:
 *
 *******************************************************************************/
void uart_block_reset(uint8_t reset_type, uint8_t port)
{
    switch (port)
    {
    case 0:
        switch (reset_type)
        {
        case UART_RESET_ALL:
            Cy_SCB_UART_ClearRxFifo(UART_0_HW);
            Cy_SCB_UART_ClearTxFifo(UART_0_HW);
            break;
        case UART_RESET_TX:
            Cy_SCB_UART_ClearTxFifo(UART_0_HW);
            break;
        case UART_RESET_RX:
            Cy_SCB_UART_ClearRxFifo(UART_0_HW);
            break;
        }
        break;

    case 1:
        switch (reset_type)
        {
        case UART_RESET_ALL:
            Cy_SCB_UART_ClearRxFifo(UART_1_HW);
            Cy_SCB_UART_ClearTxFifo(UART_1_HW);
            break;
        case UART_RESET_TX:
            Cy_SCB_UART_ClearTxFifo(UART_1_HW);
            break;
        case UART_RESET_RX:
            Cy_SCB_UART_ClearRxFifo(UART_1_HW);
            break;
        }
        break;

    case 2:
        switch (reset_type)
        {
        case UART_RESET_ALL:
            Cy_SCB_UART_ClearRxFifo(UART_2_HW);
            Cy_SCB_UART_ClearTxFifo(UART_2_HW);
            break;
        case UART_RESET_TX:
            Cy_SCB_UART_ClearTxFifo(UART_2_HW);
            break;
        case UART_RESET_RX:
            Cy_SCB_UART_ClearRxFifo(UART_2_HW);
            break;
        }
        break;

    case 3:
        switch (reset_type)
        {
        case UART_RESET_ALL:
            Cy_SCB_UART_ClearRxFifo(UART_3_HW);
            Cy_SCB_UART_ClearTxFifo(UART_3_HW);
            break;
        case UART_RESET_TX:
            Cy_SCB_UART_ClearTxFifo(UART_3_HW);
            break;
        case UART_RESET_RX:
            Cy_SCB_UART_ClearRxFifo(UART_3_HW);
            break;
        }
        break;
    }
}

/*******************************************************************************
 * Function Name: uart_block_get_status
 ********************************************************************************
 *
 * Summary:
 *  Return if the transmission was completed or not
 *
 * Parameters:
 *  port: channel to be used
 *
 * Return:
 *  Return 1 if completed
 *
 * Global variables:
 *
 *******************************************************************************/
uint8_t uart_block_get_status(uint8_t port)
{
    uint8_t status = 0;

    switch (port)
    {
    case 0:
        status = uart0_tx_dma_complete_status;
        break;
    case 1:
        status = uart1_tx_dma_complete_status;
        break;
    case 2:
        status = uart2_tx_dma_complete_status;
        break;
    case 3:
        status = uart3_tx_dma_complete_status;
        break;
    }

    return status;
}

/*******************************************************************************
 * Function Name: handle_error
 ********************************************************************************
 * Summary:
 * User defined error handling function
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */
