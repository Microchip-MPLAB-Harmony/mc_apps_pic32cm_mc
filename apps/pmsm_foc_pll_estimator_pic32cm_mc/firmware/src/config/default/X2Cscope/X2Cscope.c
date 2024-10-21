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
Headers inclusions
*******************************************************************************/
#include "X2CscopeComm.h"
#include "X2Cscope.h"

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
/**
 * @brief X2Cscope buffer, defined in x2cscope.h, used for data sent to the host.
 */
int8_t X2CscopeArray[X2CSCOPE_BUFFER_SIZE];

/**
 * @brief Compilation date and time.
 *
 * This variable holds the compilation date and time, which can be read out
 * by the "Get Device Info" X2Cscope service. The date and time are populated
 * using the `__DATE__` and `__TIME__` macros.
 */
compilationDate_t compilationDate = {__DATE__, __TIME__};

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/
/**
 * @brief Initialize the X2Cscope.
 *
 * This function sets up UART communication hooks and initializes the X2Cscope
 * with a buffer, application version, and compilation date.
 */
void X2Cscope_Init(void)
{
    /** Hook UART functions for communication with X2Cscope  */
    X2Cscope_HookUARTFunctions(sendSerial, receiveSerial, isReceiveDataAvailable, isSendReady);

    /** Initialize X2Cscope with buffer, size, version, and compilation date */
    X2Cscope_Initialise((void*)X2CscopeArray, X2CSCOPE_BUFFER_SIZE, X2CSCOPE_APP_VERSION, compilationDate);
}
