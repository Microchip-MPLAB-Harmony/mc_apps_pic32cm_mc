/**
 * Copyright (C) 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * You may use Microchip software and any derivatives exclusively with Microchip products,
 * provided you comply with these terms. It is your responsibility to ensure compliance
 * with any third-party license terms, including those for open-source software, that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTIES, WHETHER EXPRESS, IMPLIED,
 * OR STATUTORY, INCLUDING IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * MICROCHIP WILL NOT BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL, OR
 * CONSEQUENTIAL LOSS, DAMAGE, COST, OR EXPENSE ARISING FROM THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES. TO THE MAXIMUM EXTENT PERMITTED
 * BY LAW, MICROCHIP'S TOTAL LIABILITY FOR ALL CLAIMS RELATED TO THIS SOFTWARE WILL NOT
 * EXCEED THE AMOUNT YOU PAID DIRECTLY TO MICROCHIP FOR IT, IF ANY.
 */

/*******************************************************************************
 * Header Inclusions
 *******************************************************************************/
#include "X2CscopeComm.h"

/*******************************************************************************
 * Interface Functions 
 *******************************************************************************/

/**
 * @brief Sends a data byte over the UART interface.
 *
 * This function sends the specified data byte through UART. If the write operation
 * fails, it logs an error.
 *
 * @param[in] data The data byte to send.
 */
void sendSerial(uint8_t data)
{
    bool status = SERCOM3_USART_Write(&data, 1);

    if (status == false)
    {
        /** @todo Log error for failed write operation */
    }
}

/**
 * @brief Receives a data byte from the UART interface.
 *
 * This function reads a byte from the UART interface. If the read operation fails,
 * it returns 0.
 *
 * @return The received data byte, or 0 if no data is available.
 */
uint8_t receiveSerial()
{
    uint8_t data;
    bool status = SERCOM3_USART_Read(&data, 1);

    if (status == true)
    {
        return data;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief Checks if data is available in the UART receiver.
 *
 * This function returns whether the UART receiver has data available for reading.
 *
 * @return 1 if data is available, 0 otherwise.
 */
uint8_t isReceiveDataAvailable()
{
    bool status = SERCOM3_USART_ReceiverIsReady();
    return (uint8_t)status;
}

/**
 * @brief Checks if the UART transmitter is ready.
 *
 * This function returns whether the UART transmitter has space available in its
 * buffer to send more data.
 *
 * @return 1 if the transmitter is ready, 0 if the buffer is full.
 */
uint8_t isSendReady()
{
    bool status = SERCOM3_USART_TransmitterIsReady();
    return (uint8_t)status;
}
