/**
 * @brief 
 *    Source file for reference control
 *
 * @File Name 
 *    mc_reference_control.c
 *
 * @Company 
 *    Microchip Technology Inc.
 *
 * @Summary
 *    Source file which contains variables and function implementations for reference control.
 *
 * @Description
 *    This file contains variables and function implementations which are generally used for reference
 *    control in pulse width modulation. 
 */


// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

/*******************************************************************************
Headers inclusions
*******************************************************************************/
#include "mc_reference_control.h"

/*******************************************************************************
Local configuration options
*******************************************************************************/

/*******************************************************************************
 Private data types
*******************************************************************************/
/**
 * @brief Structure defining the state of the reference control module.
 */
typedef struct
{
    bool enable;                /**< Flag indicating if the module is enabled */
    bool initDone;              /**< Flag indicating if initialization is done */
    int16_t lowerLimit;         /**< Lower limit for the reference value */
    int16_t upperLimit;         /**< Upper limit for the reference value */
    int32_t scaledLowerLimit;   /**< Scaled lower limit for the reference value */
    int32_t scaledUpperLimit;   /**< Scaled upper limit for the reference value */
    int32_t scaledReference;    /**< Scaled reference value */
    int32_t scaledRampRate;     /**< Scaled ramp rate       */
    int16_t reference;          /**< Reference value */
} tmcRef_State_s;

/*******************************************************************************
Private variables
*******************************************************************************/
static tmcRef_State_s mcRef_State_mds;

/*******************************************************************************
Interface  variables
*******************************************************************************/

/*******************************************************************************
Macro Functions
*******************************************************************************/

/*******************************************************************************
Private Functions
*******************************************************************************/

/*******************************************************************************
 * Interface Functions
*******************************************************************************/

/**
 * @brief Initialize reference control module
 *
 * This function initializes the reference control module.
 *
 * @param[in] pParameters Pointer to the reference parameters structure
 */
void  mcRefI_ReferenceControlInit( tmcRef_Parameters_s * const pParameters )
{
    float32_t  temp;

    /** Link state variable structure to the module */
    pParameters->pStatePointer = (void *)&mcRef_State_mds;

    /** Set parameters */
    mcRefI_ParametersSet(pParameters);

    temp = ( K_SPEED * pParameters->minimumRpm ) + 0.5f;
    mcRef_State_mds.lowerLimit = (int16_t)temp;

    temp = ( K_SPEED * pParameters->maximumRpm ) + 0.5f;
    mcRef_State_mds.upperLimit = (int16_t)temp;

    mcRef_State_mds.scaledLowerLimit = Q_LEFT_SHIFT((int32_t)mcRef_State_mds.lowerLimit, 15u );
    mcRef_State_mds.scaledUpperLimit = Q_LEFT_SHIFT((int32_t)mcRef_State_mds.upperLimit, 15u );


    /** Update state variables */
    temp = (float32_t)( (float32_t)Q_SCALE( pParameters->rpmPerSecond / BASE_SPEED_IN_RPM ) ) /  PWM_FREQUENCY;
    mcRef_State_mds.scaledRampRate =  Q_SCALE( temp );
    mcRef_State_mds.initDone = true;
}

/**
 * @brief Enable reference control module
 *
 * This function enables the reference control module.
 *
 * @param[in] pParameters Pointer to the reference parameters structure
 */
void  mcRefI_ReferenceControlEnable( tmcRef_Parameters_s * const pParameters )
{
    /** Get the linked state variable */
    tmcRef_State_s * pState;
    pState = (tmcRef_State_s *)pParameters->pStatePointer;

    if( ( NULL == pState ) || ( !pState->initDone ))
    {
        /** Initialize parameters */
        mcRefI_ReferenceControlInit(pParameters);
    }
    else
    {
        /** For MISRA Compliance */
    }

    /** Set enable flag as true */
    pState->enable = true;
}

/**
 * @brief Disable reference control module
 *
 * This function disables the reference control module.
 *
 * @param[in] pParameters Pointer to the reference parameters structure
 */
void  mcRefI_ReferenceControlDisable( tmcRef_Parameters_s * const pParameters )
{
    /** Get the linked state variable */
    tmcRef_State_s * pState;
    pState = (tmcRef_State_s *)pParameters->pStatePointer;

    if( NULL != pState)
    {
        /** Reset state variables  */
        mcRefI_ReferenceControlReset(pParameters);
    }
    else
    {
        /** For MISRA Compliance */
    }

    /** Set enable flag as true */
    pState->enable = false;
}

/**
 * @brief Reference control
 *
 * This function performs the reference control.
 *
 * @param[in] pParameters Pointer to the reference parameters structure
 * @param[in] command The command input
 * @param[out] pOut Pointer to the output
 */
#ifdef RAM_EXECUTE
void __ramfunc__  mcRefI_ReferenceControl( tmcRef_Parameters_s * const pParameters,
                                                                       const int16_t command, int16_t * const pOut   )
#else
void mcRefI_ReferenceControl(  tmcRef_Parameters_s * const pParameters,
                                                    const int16_t command, int16_t * const pOut )
#endif
{
    /** Get the linked state variable */
    tmcRef_State_s * pState;
    pState = (tmcRef_State_s *)pParameters->pStatePointer;

    if( pState->enable )
    {
        /** Execute reference control */

        /** Execute reference control */
        int32_t scaledCommand = Q_LEFT_SHIFT((int32_t)command, 15u );
        if( (  pState->scaledReference + pState->scaledRampRate )  < scaledCommand )
        {
            pState->scaledReference += pState->scaledRampRate;
        }
        else if(  (  pState->scaledReference - pState->scaledRampRate )  > scaledCommand )
        {
            pState->scaledReference -= pState->scaledRampRate;
        }
        else
        {
            pState->scaledReference = scaledCommand;
        }

        UTIL_ApplyClampS32( &pState->scaledReference, pState->scaledUpperLimit, pState->scaledLowerLimit);
        pState->reference = (int16_t)mcUtils_RightShiftS32(pState->scaledReference, 15 );
        *pOut = pState->reference;
    }
    else
    {
        /** Reset reference control */
        mcRefI_ReferenceControlReset(pParameters);
    }
}

/**
 * @brief Reset reference control
 *
 * This function resets the reference control.
 *
 * @param[in] pParameters Pointer to the reference parameters structure
 */
void mcRefI_ReferenceControlReset( tmcRef_Parameters_s * const pParameters )
{
    /** Get the linked state variable */
    tmcRef_State_s * pState;
    pState = (tmcRef_State_s *)pParameters->pStatePointer;

    /** Reset reference control state variables  */
    pState->reference = pState->lowerLimit;
    pState->scaledReference = Q_LEFT_SHIFT((int32_t)pState->lowerLimit, 15u );
}
