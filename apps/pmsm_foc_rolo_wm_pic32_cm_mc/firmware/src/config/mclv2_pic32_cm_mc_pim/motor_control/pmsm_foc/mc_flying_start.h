/*******************************************************************************
 Flying Start Function 

  File Name:
    mc_flying_start.h

  Summary:
    Header file which contains variables and function prototypes to control the flying start 
    function

  Description:
    This file contains variables and function prototypes which are generally
    used in flying start function. It is implemented in Q2.14
    fixed Point Arithmetic.

 *******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef MCFLY_H
#define MCFLY_H

#include <stdint.h>
#include <sys/attribs.h>
#include "mc_generic_library.h"

/*******************************************************************************
 * Constants
*******************************************************************************/

/*******************************************************************************
 * User defined data-structure  
*******************************************************************************/
typedef enum 
{
    FlyingStartState_Detect, 
    FlyingStartState_Decide,
    FlyingStartState_PassiveBrake,
    FlyingStartState_ActiveBrake,
    FlyingStartState_Finish
}tmcFls_FlyingStartState_e;

typedef struct 
{
    int16_t *  estimatedSpeed;
    int16_t *  commandDirection;  
    int16_t *  idFeedback;
    int16_t *  iqFeedback;
    int16_t *  speedFeedback;
    tmcLib_PiController_s * idController;
    tmcLib_PiController_s * iqController;
    tmcLib_PiController_s * speedController;
}tmcFls_InputSignal_s;

typedef struct 
{
    uint16_t flyingStartDetectCount;
    int16_t flyingStartMinimumSpeed;
    int16_t flyingStartAlignTransCurrent;
}tmcFls_Parameters_s;

typedef struct 
{
    tmcFls_FlyingStartState_e flyingStartState;
    uint16_t flyingStartCounter;
    
}tmcFls_StateSignal_s;

typedef struct 
{
    int16_t minimumSpeedinRpm;
    float flyingStartDecideTimeInSec;  
    float flyingStartAlignTransCurrent;
    tmcFls_InputSignal_s inPort;
}tmcFls_ConfigParameters_s;

/*******************************************************************************
 * Interface Variables   
*******************************************************************************/


/*******************************************************************************
 * Interface Functions  
*******************************************************************************/



#endif // _MCFLY_H
