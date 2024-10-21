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

#ifndef X2CSCOPE_H
#define X2CSCOPE_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
Headers inclusions
*******************************************************************************/

#include <stdint.h>
#include "stdbool.h"


/** 
 * @def X2CSCOPE_BUFFER_SIZE
 * @brief X2Cscope buffer size in bytes, uses the specified number of bytes from RAM.
 */
#ifndef X2CSCOPE_BUFFER_SIZE
#define X2CSCOPE_BUFFER_SIZE   5000
#endif

/** 
 * @def X2CSCOPE_APP_VERSION
 * @brief X2Cscope application version (2 bytes).
 */
#ifndef X2CSCOPE_APP_VERSION
#define X2CSCOPE_APP_VERSION  1
#endif

/**
 * @brief Structure to hold compilation date and time.
 */
typedef const struct compilationDate_type {
    uint8_t date[11];  /**< Compilation date in format YYYY-MM-DD */
    uint8_t time[8];   /**< Compilation time in format HH:MM:SS */
} compilationDate_t;

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/
/**
 * @brief Initialise the X2Cscope buffer and LNET protocol.
 * 
 * @param scopeArray Pointer to the scope array to be used.
 * @param scopeSize Size of the scope array.
 * @param appVersion Version of the X2Cscope application.
 * @param compilationDate Compilation date and time structure.
 */
void X2Cscope_Initialise(void* scopeArray, uint16_t scopeSize, uint16_t appVersion, compilationDate_t compilationDate);

/**
 * @brief Initialise X2Cscope and hook communication interfaces.
 */
void X2Cscope_Init(void);

/**
 * @brief Handle communication with X2Cscope GUI on the PC side.
 * 
 * This function should be called in the idle loop.
 */
void X2Cscope_Communicate(void);

/**
 * @brief Sample point of the scope functionality.
 * 
 * This function should be called at fixed periods.
 */
void X2Cscope_Update(void);

#ifdef __cplusplus
}

#endif

#endif /* X2CSCOPE_H */
