/*******************************************************************************
 Generic Motor Control Library 

  Company:
    Microchip Technology Inc.

  File Name:
    mc_function_test.c

  Summary:
    Functional test coordination

  Description:
   Functional Test Coordination
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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
 * Include Files
*******************************************************************************/
#include <stdint.h>
#include "mc_function_test.h"

/*******************************************************************************
 * Interface Variables
*******************************************************************************/
typedef enum 
{
    mcTst_StartStopCheck,
    mcTst_ReverseRotationCheck,
    mcTst_OpenLoopCheck,
    mcTst_CurrentLoopCheck,
    mcTst_SpeedLoopCheck,
    mcTst_FieldWeakeningCheck,
}tmcTst_FunctionTestCase_e;

int16_t mcTst_FunctionalTestTrigger_gds16;
tmcTst_FunctionTestCase_e mcTst_FunctionalTestCase_gds16;

/*******************************************************************************
 * Local/ Private Variables
*******************************************************************************/


/*******************************************************************************
 * Local/ Private Functions
*******************************************************************************/

/*******************************************************************************
 * Interface Functions
*******************************************************************************/
void mcTst_FunctionalTestRun( void )
{
    if( mcTst_FunctionalTestTrigger_gds16 )
    {
        switch( mcTst_FunctionalTestCase_gds16)
        {
            case mcTst_StartStopCheck:
            {
                
            }
            break;
            
            case mcTst_ReverseRotationCheck:
            {
                
            }
            break;
            
            case mcTst_OpenLoopCheck:
            {
                
            }
            break;
            
            case mcTst_CurrentLoopCheck:
            {
                
            }
            break;
            
            case mcTst_SpeedLoopCheck:
            {
                
            }
            break;
            
            case mcTst_FieldWeakeningCheck:
            {
                
            }
            break;
            
            default:
            {
                
            }         
        }
    }
}
/* *****************************************************************************
 End of File
 */
