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

#ifndef X2CSCOPECOMMUNICATION_H
#define X2CSCOPECOMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
Headers inclusions
*******************************************************************************/

#include <stdint.h>
#include "stdbool.h"
#include "definitions.h"

/*******************************************************************************
 Type Definitions
 *******************************************************************************/
typedef void (*SERIAL_SEND)( uint8_t data );
typedef uint8_t (*SERIAL_RECEIVE)( void );
typedef uint8_t (*SERIAL_DATA_AVAILABLE)( void );
typedef uint8_t (*SERIAL_SEND_READY)( void );

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/

/* MISRA C-2012 8.6 deviated below. Deviation record ID - H3_MISRAC_2012_R_8_6_DR_1 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma coverity compliance block deviate:2 "MISRA C-2012 Rule 8.6" "H3_MISRAC_2012_R_8_6_DR_1"
/**
 * @brief Hook UART communication functions for X2Cscope.
 * 
 * This function hooks the necessary UART functions for communication between 
 * X2Cscope and the external device. These functions are called during the 
 * operation of X2Cscope to send and receive data over a UART interface.
 * 
 * @param sendSerialFcnPntr Pointer to the function for sending a serial byte.
 * @param receiveSerialFcnPntr Pointer to the function for receiving a serial byte.
 * @param isReceiveDataAvailableFcnPntr Pointer to the function to check if data is available.
 * @param isSendReadyFcnPntr Pointer to the function to check if the send buffer is ready.
 */
void X2Cscope_HookUARTFunctions( SERIAL_SEND sendSerialFcn, SERIAL_RECEIVE receiveSerialFcn,
                                                         SERIAL_DATA_AVAILABLE isReceiveDataAvailableFcn, 
                                                         SERIAL_SEND_READY isSendReadyFcn );

/**
 * @brief Send a byte over UART.
 * 
 * This function should be implemented by the user. It is typically defined 
 * in X2CscopeComm.c and is used to send data from the X2Cscope to the host device.
 * 
 * @param data The byte to send over the UART.
 */
void sendSerial(uint8_t data);

/**
 * @brief Receive a byte over UART.
 * 
 * This function should be implemented by the user. It is typically defined 
 * in X2CscopeComm.c and is used to receive data from the host device.
 * 
 * @return The byte received over the UART.
 */
uint8_t receiveSerial(void);

/**
 * @brief Check if UART receive data is available.
 * 
 * This function should be implemented by the user. It is typically defined 
 * in X2CscopeComm.c and is used to check if there is data available to receive.
 * 
 * @return 1 if data is available, 0 otherwise.
 */
uint8_t isReceiveDataAvailable(void);

/**
 * @brief Check if UART is ready to send data.
 * 
 * This function should be implemented by the user. It is typically defined 
 * in X2CscopeComm.c and is used to check if the UART is ready to send data.
 * 
 * @return 1 if the UART is ready to send, 0 otherwise.
 */
uint8_t isSendReady(void);

#pragma coverity compliance end_block "MISRA C-2012 Rule 8.6"
#pragma GCC diagnostic pop
/* MISRAC 2012 deviation block end */

#ifdef __cplusplus
}
#endif

#endif	/* X2CSCOPECOMMUNICATION_H */