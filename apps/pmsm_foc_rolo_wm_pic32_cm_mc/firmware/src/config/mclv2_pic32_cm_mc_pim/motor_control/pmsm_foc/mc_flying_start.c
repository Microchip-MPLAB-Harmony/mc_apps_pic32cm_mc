/*******************************************************************************
 Flying Start Function
 
  Company:
    Microchip Technology Inc.

  File Name:
    mc_flying_start.c

  Summary:
    Flying start functions.

  Description:
    This file implements flying start function.
 
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2021 Microchip Technology Inc. and its subsidiaries.
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

#include <stdlib.h>
#include "definitions.h"
#include "math.h"
#include "mc_flying_start.h"
#include "mc_hardware_abstraction.h"

/*******************************************************************************
 * Module variables 
*******************************************************************************/
static tmcFls_InputSignal_s     mcFls_InputSignal_mds;
static tmcFls_Parameters_s      mcFls_Parameters_mds;
static tmcFls_StateSignal_s     mcFls_StateSignal_mds;

/*******************************************************************************
 * Interface variables 
*******************************************************************************/

/*******************************************************************************
 * Local Functions  
*******************************************************************************/
/*! \brief Read input signals 
 * 
 * Details.
 * Read input signal
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */

/*******************************************************************************
 * Interface Functions  
*******************************************************************************/

/*! \brief Initialize flying start function 
 * 
 * Details.
 * Initialize flying start function
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcFls_FlyingStartInitialize(const  tmcFls_ConfigParameters_s * const configParam )
{
    mcFls_Parameters_mds.flyingStartDetectCount = (uint16_t)( (float)configParam->flyingStartDecideTimeInSec 
                                                                                                * (float)CURRENT_CONTROL_FREQUENCY );
    
    mcFls_Parameters_mds.flyingStartMinimumSpeed = (int16_t)( 2.0f * M_PI * NUM_POLE_PAIRS 
                                                                                                  * configParam->minimumSpeedinRpm * K_SPEED /60.0f);
    
    mcFls_Parameters_mds.flyingStartAlignTransCurrent = (int16_t)( configParam->flyingStartAlignTransCurrent
                                                                                                      * CURRENT_CONTROL_FREQUENCY );
    
    mcFls_InputSignal_mds = configParam->inPort;
}

/*! \brief Run flying start function 
 * 
 * Details.
 * Run flying start function
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
tStd_ReturnType_e mcFls_FlyingStartControl( void )
{
    tStd_ReturnType_e returnStatus = returnType_Running;
            
    /* Flying Start timer */
    switch(mcFls_StateSignal_mds.flyingStartState )
    {
        case FlyingStartState_Detect:
        {
            uint32_t dutyCycle[3] = { 0u, 0u, 0u }; 
            mcFls_StateSignal_mds.flyingStartCounter++;
                                   
             /* Turn on low-side FETs */
             mcHal_InverterDutySet(dutyCycle );
             
             if( mcFls_Parameters_mds.flyingStartDetectCount < mcFls_StateSignal_mds.flyingStartCounter )
             {
                 mcFls_StateSignal_mds.flyingStartCounter = 0u;
                 mcFls_StateSignal_mds.flyingStartState = FlyingStartState_Decide;
             }
        }
        break;
        
        case FlyingStartState_Decide:
        {                    
            /* Check if commanded and detected direction are opposite */
            if(( ( *mcFls_InputSignal_mds.commandDirection ) ^ (*mcFls_InputSignal_mds.estimatedSpeed ) ) & 0x1000 )
            {
                 mcFls_StateSignal_mds.flyingStartState = FlyingStartState_PassiveBrake;
            }
            else
            {
                int16_t absEstimatedSpeed;
                absEstimatedSpeed = abs( *mcFls_InputSignal_mds.estimatedSpeed );
                if( mcFls_Parameters_mds.flyingStartMinimumSpeed < absEstimatedSpeed )
                {
                    /* Speed Controller in manual mode*/
                    mcFls_InputSignal_mds.speedController->reference = 0;
                    mcFls_InputSignal_mds.speedController->feedback = *mcFls_InputSignal_mds.speedFeedback;
                    mcLib_PiControllerManualModeRun(mcFls_InputSignal_mds.speedController );

                    /* D axis Current Controller in manual mode */
                    mcFls_InputSignal_mds.idController->reference = 0;
                    mcFls_InputSignal_mds.idController->feedback = *mcFls_InputSignal_mds.idFeedback;
                    mcLib_PiControllerManualModeRun(mcFls_InputSignal_mds.idController );

                     /* Q axis Current Controller in manual mode */
                    mcFls_InputSignal_mds.iqController->reference = 0;
                    mcFls_InputSignal_mds.iqController->feedback = *mcFls_InputSignal_mds.iqFeedback;
                    mcLib_PiControllerManualModeRun(mcFls_InputSignal_mds.iqController );

                    returnStatus = returnType_Passed;
                }
                else
                {
                    mcFls_StateSignal_mds.flyingStartState = FlyingStartState_PassiveBrake;
                }
            }
        }
        break;
        
        case FlyingStartState_PassiveBrake:
        {
            if( mcFls_Parameters_mds.flyingStartAlignTransCurrent > ( *mcFls_InputSignal_mds.iqFeedback ) )
            {
                uint32_t dutyCycle[3] = { 0u, 0u, 0u }; 
                mcFls_StateSignal_mds.flyingStartCounter++;
                                   
                /* Turn on low-side FETs */
                mcHal_InverterDutySet(dutyCycle );
            }
            else
            {
                /* Speed Controller in manual mode*/
                mcFls_InputSignal_mds.speedController->reference = 0;
                mcFls_InputSignal_mds.speedController->feedback = 0;
                mcLib_PiControllerManualModeRun(mcFls_InputSignal_mds.speedController );

                /* D axis Current Controller in manual mode */
                mcFls_InputSignal_mds.idController->reference = 0;
                mcFls_InputSignal_mds.idController->feedback = 0;
                mcLib_PiControllerManualModeRun(mcFls_InputSignal_mds.idController );

                /* Q axis Current Controller in manual mode */
                mcFls_InputSignal_mds.iqController->reference = 0;
                mcFls_InputSignal_mds.iqController->feedback = 0;
                mcLib_PiControllerManualModeRun(mcFls_InputSignal_mds.iqController );

                 returnStatus = returnType_Failed;
            }
        }
        break;
        
        case FlyingStartState_ActiveBrake:
        {
             returnStatus = returnType_Failed;
        }
        break;
        
        default:
        {
            /* Should not come here */
        }
        break;
    }  
    
    return returnStatus;
}


/*! \brief Reset  flying start function 
 * 
 * Details.
 * Reset flying start function enable 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcFls_FlyingStartReset( void )
{
    
}
