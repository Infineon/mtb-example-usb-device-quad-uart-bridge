/******************************************************************************
* File Name: quad_serial.c
*
* Description: This file contains source code that controls Quad serial interface.
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

#include "quad_serial.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <stdio.h>
#include "USB.h"
#include "USB_Bulk.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
/* Handle to a valid BULK instance*/
static USB_BULK_HANDLE handle_inst[CY_NUM_PORTS];

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static uint16_t quad_serial_strlen(const char string[]);
static void usb_add_bulk(void);

/*********************************************************************
 *       Information that are used during enumeration
 **********************************************************************/
static const USB_DEVICE_INFO usb_device_info = {
    0x058B,                  /* VendorId    */
    0x027D,                  /* ProductId    */
    "Infineon Technologies", /* VendorName   */
    "Quad USB to UART",      /* ProductName  */
    "12345678"               /* SerialNumber */
};

/*******************************************************************************
 * Function Name: quad_serial_init
 ********************************************************************************
 *
 * Summary:
 *  This function initialize the Quad Serial interface to be ready for the
 *  receive data from the PC.
 *
 * Parameters:
 *  None.
 *
 * Return:
 *  None.
 *
 * Global variables:
 *   cyLineChanged: Initialized to zero.
 *   cyOutEP: Used as an OUT endpoint number.
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
void quad_serial_init(void)
{
    /* Initializes the USB stack */
    USBD_Init();

    /* Enables combination of multi-interface device classes */
    USBD_EnableIAD();

    /* Endpoint Initialization for CDC class */
    usb_add_bulk();

    /* Set device info used in enumeration */
    USBD_SetDeviceInfo(&usb_device_info);

    /* Start the USB stack */
    USBD_Start();

    /* Wait for configuration */
    while ((USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) != USB_STAT_CONFIGURED)
    {
        cyhal_gpio_toggle(CYBSP_USER_LED);
        cyhal_system_delay_ms(USB_CONFIG_DELAY);
    }

    /* Turning the LED on to indicate device is active */
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
}

/*********************************************************************
 * Function Name: usb_add_bulk
 **********************************************************************
 * Summary:
 *  Add Bulk interface to USB stack
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 **********************************************************************/
static void usb_add_bulk(void)
{
    static U8 out_buffer[CY_NUM_PORTS][CY_TX_EP_BUFFER_SIZE];
    USB_BULK_INIT_DATA_EX init_data;
    USB_ADD_EP_INFO ep_in;
    USB_ADD_EP_INFO ep_out;
    uint8_t interface_index = 0;

    interface_index = 0;
    memset(&init_data, 0, sizeof(init_data));
    ep_in.Flags = 0;                             // Flags not used.
    ep_in.InDir = USB_DIR_IN;                    // IN direction (Device to Host)
    ep_in.Interval = 0;                          // Interval not used for Bulk endpoints.
    ep_in.MaxPacketSize = CY_RX_EP_BUFFER_SIZE;  // Maximum packet size (64 for Bulk in full-speed).
    ep_in.TransferType = USB_TRANSFER_TYPE_BULK; // Endpoint type - Bulk.
    init_data.EPIn = USBD_AddEPEx(&ep_in, NULL, 0);

    ep_out.Flags = 0;                             // Flags not used.
    ep_out.InDir = USB_DIR_OUT;                   // OUT direction (Host to Device)
    ep_out.Interval = 0;                          // Interval not used for Bulk endpoints.
    ep_out.MaxPacketSize = CY_TX_EP_BUFFER_SIZE;  // Maximum packet size (64 for Bulk in full-speed).
    ep_out.TransferType = USB_TRANSFER_TYPE_BULK; // Endpoint type - Bulk.
    init_data.EPOut = USBD_AddEPEx(&ep_out, out_buffer[interface_index], sizeof(out_buffer) / CY_NUM_PORTS);

    init_data.pInterfaceName = "Bulk Interface 0";
    handle_inst[interface_index] = USBD_BULK_Add_Ex(&init_data);
    USBD_BULK_SetMSDescInfo(handle_inst[interface_index]);

    interface_index = 1;
    memset(&init_data, 0, sizeof(init_data));
    ep_in.Flags = 0;                             // Flags not used.
    ep_in.InDir = USB_DIR_IN;                    // IN direction (Device to Host)
    ep_in.Interval = 0;                          // Interval not used for Bulk endpoints.
    ep_in.MaxPacketSize = CY_RX_EP_BUFFER_SIZE;  // Maximum packet size (64 for Bulk in full-speed).
    ep_in.TransferType = USB_TRANSFER_TYPE_BULK; // Endpoint type - Bulk.
    init_data.EPIn = USBD_AddEPEx(&ep_in, NULL, 0);

    ep_out.Flags = 0;                             // Flags not used.
    ep_out.InDir = USB_DIR_OUT;                   // OUT direction (Host to Device)
    ep_out.Interval = 0;                          // Interval not used for Bulk endpoints.
    ep_out.MaxPacketSize = CY_TX_EP_BUFFER_SIZE;  // Maximum packet size (64 for Bulk in full-speed).
    ep_out.TransferType = USB_TRANSFER_TYPE_BULK; // Endpoint type - Bulk.
    init_data.EPOut = USBD_AddEPEx(&ep_out, out_buffer[interface_index], sizeof(out_buffer) / CY_NUM_PORTS);

    init_data.pInterfaceName = "Bulk Interface 1";
    handle_inst[interface_index] = USBD_BULK_Add_Ex(&init_data);
    USBD_BULK_SetMSDescInfo(handle_inst[interface_index]);

    interface_index = 2;
    memset(&init_data, 0, sizeof(init_data));
    ep_in.Flags = 0;                             // Flags not used.
    ep_in.InDir = USB_DIR_IN;                    // IN direction (Device to Host)
    ep_in.Interval = 0;                          // Interval not used for Bulk endpoints.
    ep_in.MaxPacketSize = CY_RX_EP_BUFFER_SIZE;  // Maximum packet size (64 for Bulk in full-speed).
    ep_in.TransferType = USB_TRANSFER_TYPE_BULK; // Endpoint type - Bulk.
    init_data.EPIn = USBD_AddEPEx(&ep_in, NULL, 0);

    ep_out.Flags = 0;                             // Flags not used.
    ep_out.InDir = USB_DIR_OUT;                   // OUT direction (Host to Device)
    ep_out.Interval = 0;                          // Interval not used for Bulk endpoints.
    ep_out.MaxPacketSize = CY_TX_EP_BUFFER_SIZE;  // Maximum packet size (64 for Bulk in full-speed).
    ep_out.TransferType = USB_TRANSFER_TYPE_BULK; // Endpoint type - Bulk.
    init_data.EPOut = USBD_AddEPEx(&ep_out, out_buffer[interface_index], sizeof(out_buffer) / CY_NUM_PORTS);

    init_data.pInterfaceName = "Bulk Interface 2";
    handle_inst[interface_index] = USBD_BULK_Add_Ex(&init_data);
    USBD_BULK_SetMSDescInfo(handle_inst[interface_index]);

    interface_index = 3;
    memset(&init_data, 0, sizeof(init_data));
    ep_in.Flags = 0;                             // Flags not used.
    ep_in.InDir = USB_DIR_IN;                    // IN direction (Device to Host)
    ep_in.Interval = 0;                          // Interval not used for Bulk endpoints.
    ep_in.MaxPacketSize = 32;                    // Maximum packet size (64 for Bulk in full-speed).        
    ep_in.TransferType = USB_TRANSFER_TYPE_BULK; // Endpoint type - Bulk.
    init_data.EPIn = USBD_AddEPEx(&ep_in, NULL, 0);

    ep_out.Flags = 0;                             // Flags not used.
    ep_out.InDir = USB_DIR_OUT;                   // OUT direction (Host to Device)
    ep_out.Interval = 0;                          // Interval not used for Bulk endpoints.
    ep_out.MaxPacketSize = CY_TX_EP_BUFFER_SIZE;  // Maximum packet size (64 for Bulk in full-speed).
    ep_out.TransferType = USB_TRANSFER_TYPE_BULK; // Endpoint type - Bulk.
    init_data.EPOut = USBD_AddEPEx(&ep_out, out_buffer[interface_index], sizeof(out_buffer) / CY_NUM_PORTS);

    init_data.pInterfaceName = "Bulk Interface 3";
    handle_inst[interface_index] = USBD_BULK_Add_Ex(&init_data);
    USBD_BULK_SetMSDescInfo(handle_inst[interface_index]);
}

/*******************************************************************************
 * Function Name: quad_serial_put_data
 ********************************************************************************
 *
 * Summary:
 *  This function sends a specified number of bytes from the location specified
 *  by a pointer to the PC. The quad_serial_is_ready() function should be
 *  called before sending new data, to be sure that the previous data has
 *  finished sending.
 *  If the last sent packet is less than maximum packet size the USB transfer
 *  of this short packet will identify the end of the segment. If the last sent
 *  packet is exactly maximum packet size, it shall be followed by a zero-length
 *  packet (which is a short packet) to assure the end of segment is properly
 *  identified. To send zero-length packet, use quad_serial_put_data() API
 *  with length parameter set to zero.
 *
 * Parameters:
 *  data_ptr: pointer to the buffer containing data to be sent.
 *  length: Specifies the number of bytes to send from the data_ptr
 *  buffer. Maximum length will be limited by the maximum packet
 *  size for the endpoint. Data will be lost if length is greater than Max
 *  Packet Size.
 *
 * Return:
 *  None.
 *
 * Global variables:
 *   cyInEP: QuadSerial IN endpoint number used for sending data.
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
void quad_serial_put_data(const uint8_t *data_ptr, uint16_t length, uint8_t port)
{
    if (port < CY_NUM_PORTS)
    {
        /* Limits length to maximum packet size for the EP + Port Status */
        if (length > (CY_TX_EP_BUFFER_SIZE - 2))
        {
            /* Caution: Data will be lost if length is greater than Max Packet Length */
            length = (CY_TX_EP_BUFFER_SIZE - 2);
        }

        USBD_BULK_Write(handle_inst[port], data_ptr, length, -1);
    }
}

/*******************************************************************************
 * Function Name: quad_serial_strlen
 ********************************************************************************
 *
 * Summary:
 *  Calculates length of a null terminated string.
 *
 * Parameters:
 *  string: pointer to the string.
 *
 * Return:
 *  Length of the string
 *
 *******************************************************************************/
static uint16_t quad_serial_strlen(const char string[])
{
    uint16_t len = 0u;

    while (string[len] != (char)0)
    {
        len++;
    }

    return (len);
}

/*******************************************************************************
 * Function Name: quad_serial_put_string
 ********************************************************************************
 *
 * Summary:
 *  This function sends a null terminated string to the PC. This function will
 *  block if there is not enough memory to place the whole string. It will block
 *  until the entire string has been written to the transmit buffer.
 *  The quad_serial_is_ready() function should be called before sending data with
 *  a new call to quad_serial_put_string(), to be sure that the previous data
 *  has finished sending.
 *
 * Parameters:
 *  string: pointer to the string to be sent to the PC.
 *
 * Return:
 *  None.
 *
 * Global variables:
 *   cyInEP: QuadSerial IN endpoint number used for sending data.
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
void quad_serial_put_string(const char string[], uint8_t port)
{
    uint16_t str_length;
    uint16_t send_length;
    uint16_t buf_index = 0u;

    if (port < CY_NUM_PORTS)
    {
        /* Get length of the null terminated string */
        str_length = quad_serial_strlen(string);

        do
        {
            /* Limits length to maximum packet size for the EP + 2 bytes Port status */
            send_length = (str_length > (CY_TX_EP_BUFFER_SIZE - 2)) ? (CY_TX_EP_BUFFER_SIZE - 2) : str_length;
            /* Enable IN transfer */
            USBD_BULK_Write(handle_inst[port], (const uint8_t *)&string[buf_index], send_length, 0);

            str_length -= send_length;

            /* If more data are present to send or full packet was sent */
            if ((str_length > 0u) || (send_length == (CY_TX_EP_BUFFER_SIZE - 2)))
            {
                buf_index += send_length;

                /* Waits for specified number of bytes to be written to host */
                USBD_BULK_WaitForTXReady(handle_inst[port], 0);

                /* If the last sent packet is exactly maximum packet size,
                 *  it shall be followed by a zero-length packet to assure the
                 *  end of segment is properly identified by the terminal.
                 */
                if (str_length == 0u)
                {
                    USBD_BULK_Write(handle_inst[port], NULL, 0u, 0u);

                    /* Waits for specified number of bytes to be written to host */
                    USBD_BULK_WaitForTXReady(handle_inst[port], 0);
                }
            }
        } while (str_length > 0u);
    }
}

/*******************************************************************************
 * Function Name: quad_serial_put_char
 ********************************************************************************
 *
 * Summary:
 *  Writes a single character to the PC.
 *
 * Parameters:
 *  tx_data_byte: Character to be sent to the PC.
 *
 * Return:
 *  None.
 *
 * Global variables:
 *   cyInEP: QuadSerial IN endpoint number used for sending data.
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
void quad_serial_put_char(char tx_data_byte, uint8_t port)
{
    uint8_t data_byte;
    data_byte = (uint8_t)tx_data_byte;

    USBD_BULK_Write(handle_inst[port], &data_byte, 1u, 0u);
}

/*******************************************************************************
 * Function Name: quad_serial_put_crlf
 ********************************************************************************
 *
 * Summary:
 *  Sends a carriage return (0x0D) and line feed (0x0A) to the PC
 *
 * Parameters:
 *  None.
 *
 * Return:
 *  None.
 *
 * Global variables:
 *   cyInEP: QuadSerial IN endpoint number used for sending data.
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
void quad_serial_put_crlf(uint8_t port)
{
    const uint8_t tx_data[2] = {0x0Du, 0x0Au};

    USBD_BULK_Write(handle_inst[port], (const uint8_t *)tx_data, 2u, 0u);
}

/*******************************************************************************
 * Function Name: quad_serial_data_is_ready
 ********************************************************************************
 *
 * Summary:
 *  Returns a nonzero value if the component received data or received
 *  zero-length packet. The quad_serial_get_all() or
 *  quad_serial_get_data() API should be called to read data from the buffer
 *  and re-init OUT endpoint even when zero-length packet received.
 *
 * Parameters:
 *  None.
 *
 * Return:
 *  If the OUT packet received this function returns a nonzero value.
 *  Otherwise zero is returned.
 *
 * Global variables:
 *   cyOutEP: QuadSerial OUT endpoint number used.
 *
 *******************************************************************************/
uint8_t quad_serial_data_is_ready(uint8_t port)
{
    if (USBD_BULK_GetNumBytesInBuffer(handle_inst[port]) > 0u)
        return 1;
    else
        return 0;
}

/*******************************************************************************
 * Function Name: quad_serial_is_ready
 ********************************************************************************
 *
 * Summary:
 *  This function returns a nonzero value if the component is ready to send more
 *  data to the PC; otherwise, it returns zero. The function should be called
 *  before sending new data when using any of the following APIs:
 *  quad_serial_put_data(),quad_serial_put_string(),
 *  quad_serial_put_char or quad_serial_put_crlf(),
 *  to be sure that the previous data has finished sending.
 *
 * Parameters:
 *  None.
 *
 * Return:
 *  If the buffer can accept new data, this function returns a nonzero value.
 *  Otherwise, it returns zero.
 *
 * Global variables:
 *   cyInEP: CDC IN endpoint number used.
 *
 *******************************************************************************/
uint8_t quad_serial_is_ready(uint8_t port)
{
    uint8_t status = 0u;

    if (USBD_BULK_GetNumBytesRemToWrite(handle_inst[port]) == 0)
        status = 1u;
    else
        status = 0u;

    return status;
}

/*******************************************************************************
 * Function Name: quad_serial_get_data
 ********************************************************************************
 *
 * Summary:
 *  This function gets a specified number of bytes from the input buffer and
 *  places them in a data array specified by the passed pointer.
 *  The quad_serial_data_is_ready() API should be called first, to be sure
 *  that data is received from the host. If all received data will not be read at
 *  once, the unread data will be lost. The quad_serial_get_data() API should
 *  be called to get the number of bytes that were received.
 *
 * Parameters:
 *  data_ptr: Pointer to the data array where data will be placed.
 *  Length: Number of bytes to read into the data array from the RX buffer.
 *          Maximum length is limited by the the number of received bytes.
 *
 * Return:
 *  Number of bytes received.
 *
 * Global variables:
 *   cyOutEP: QuadSerial OUT endpoint number used.
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
uint16_t quad_serial_get_data(uint8_t *data_ptr, uint16_t length, uint8_t port)
{
    return (USBD_BULK_Receive(handle_inst[port], data_ptr, length, -1));
}

/*******************************************************************************
 * Function Name: quad_serial_get_all
 ********************************************************************************
 *
 * Summary:
 *  Gets all bytes of received data from the input buffer and places it into a
 *  specified data array. quad_serial_data_is_ready() API should be called
 *  before, to be sure that data is received from the Host.
 *
 * Parameters:
 *  data_ptr: Pointer to the data array where data will be placed.
 *
 * Return:
 *  Number of bytes received.
 *
 * Global variables:
 *   cyOutEP: QuadSerial OUT endpoint number used.
 *   USBFS_EP[].bufferSize: EP max packet size is used as a length
 *     to read all data from the EP buffer.
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
uint16_t quad_serial_get_all(uint8_t *data_ptr, uint8_t port)
{
    return (USBD_BULK_Read(handle_inst[port], data_ptr, CY_RX_EP_BUFFER_SIZE, 1));
}

/*******************************************************************************
 * Function Name: quad_serial_get_char
 ********************************************************************************
 *
 * Summary:
 *  This function reads one byte of received data from the buffer. If more than
 *  one byte has been received from the host, the rest of the data will be lost.
 *
 * Parameters:
 *  None.
 *
 * Return:
 *  Received one character.
 *
 * Global variables:
 *   cyOutEP: QuadSerial OUT endpoint number used.
 *
 * Reentrant:
 *  No.
 *
 *******************************************************************************/
uint8_t quad_serial_get_char(uint8_t port)
{
    uint8_t rx_data;
    USBD_BULK_Receive(handle_inst[port], &rx_data, 1u, 1u);
    return (rx_data);
}

/* [] END OF FILE */
