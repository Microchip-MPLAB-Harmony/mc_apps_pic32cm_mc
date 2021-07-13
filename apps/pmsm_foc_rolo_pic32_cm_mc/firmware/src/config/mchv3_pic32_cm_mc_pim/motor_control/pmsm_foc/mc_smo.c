/*******************************************************************************
 Motor Control Coordination Functions 

  Company:
    Microchip Technology Inc.

  File Name:
    mc_infrastructure.c

  Summary:
    Motor Control coordination functions.

  Description:
    This file implements functions for motor control coordination.
 
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
Headers inclusions
*******************************************************************************/

#include "mc_GenericLibrary.h"
#include "definitions.h"
#include "mc_FunctionCoordinator.h"
#include "mc_pwm.h"
#include "mc_smo.h"
#include "mc_rolo.h"
#include "mc_CurrentMeasurement.h"
#include "userparams.h"

/******************************************************************************/
/*                          FUNCTION MACROS                                   */
/******************************************************************************/
#define SIGNUM(a)  ( a > 0 )?1:-1

/******************************************************************************/
/*                          INTERFACE VARIABLES                               */
/******************************************************************************/

tMCEST_INPUT_SIGNAL_S  gMCEST_InputSignal;
tMCEST_PARAMETERS_S    gMCEST_Parameters;
tMCEST_STATE_SIGNAL_S  gMCEST_StateSignal;
tMCEST_OUTPUT_SIGNAL_S gMCEST_OutputSignal;

/******************************************************************************/
/*                           LOCAL FUNCTIONS                                  */
/******************************************************************************/
uint16_t offset = 0;
__STATIC_INLINE void MCRPO_ReadInputSignals( void )
{
    gMCEST_InputSignal.Ua = gMCPWM_OutputSignal.Uab.a;
    gMCEST_InputSignal.Ub = gMCPWM_OutputSignal.Uab.b;
    gMCEST_InputSignal.ia = gMCCUR_OutputSignal.clarksCurrent.a;
    gMCEST_InputSignal.ib = gMCCUR_OutputSignal.clarksCurrent.b;
}

__STATIC_INLINE void MCRPO_WriteOutputSignals( void )
{
    /* Update speed output */
    gMCEST_OutputSignal.speed = gMCEST_StateSignal.speed;
   
    /* Delay compensation for angle */
    gMCEST_OutputSignal.angle = gMCEST_StateSignal.angle + offset;
    
}

/******************************************************************************/
/*                          INTERFACE FUNCTIONS                               */
/******************************************************************************/

__STATIC_INLINE int32_t MCLIB_Multiplication( int32_t a, IQ b )
{
    return ( ( a * b.val ) >> b.shr ); 
}

/******************************************************************************/
/* Function name: MCRPO_InitializeEstimator                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* loosing of control of the phase estimation algorithm check                 */
/******************************************************************************/
void MCEST_InitializeEstimator( void )
{
    /* Initialize PLL parameters */
    MCLIB_FloatToIQ( CURRENT_OBSERVER_SYSTEM_MATRIX_G11, &gMCEST_Parameters.g11 );
    MCLIB_FloatToIQ( CURRENT_OBSERVER_SYSTEM_MATRIX_G22, &gMCEST_Parameters.g22 );
    MCLIB_FloatToIQ( CURRENT_OBSERVER_INPUT_MATRIX_H11,  &gMCEST_Parameters.h11 );
    MCLIB_FloatToIQ( CURRENT_OBSERVER_INPUT_MATRIX_H11,  &gMCEST_Parameters.h22 );
    MCLIB_FloatToIQ( CURRENT_OBSERVER_SLIDE_MATRIX_B11,  &gMCEST_Parameters.b11 );
    MCLIB_FloatToIQ( CURRENT_OBSERVER_SLIDE_MATRIX_B22,  &gMCEST_Parameters.b22);
 
    /* Initialize state variables */
    gMCEST_Parameters.m = MCLIB_Multiplication( (int32_t)SLIDING_MODE_GAIN, gMCEST_Parameters.b11 );
     
    gMCEST_Parameters.boundary = CURRENT_OBSERVER_BOUNDARY_LAYER_IN_INTERNAL_UNIT;
    gMCEST_Parameters.mdbi = ( gMCEST_Parameters.m  * BASE_VALUE )/ gMCEST_Parameters.boundary;

    gMCEST_Parameters.filtParam.val = 1200;
    gMCEST_Parameters.filtParam.shr = 14;
      
}



__STATIC_INLINE int32_t MCLIB_SignumFunction( int32_t input )
{
    int32_t s32a;
    
    if( gMCEST_Parameters.boundary < input )
    {
        s32a =  gMCEST_Parameters.m;
    }
    else if( -gMCEST_Parameters.boundary > input)
    {
        s32a = -gMCEST_Parameters.m;
    }
    else 
    {
        s32a = ( gMCEST_Parameters.mdbi * input ) >> 14;
    }
    return s32a;
}

/******************************************************************************/
/* Function name: MCRPO_PositionSpeedCalculation                              */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* loosing of control of the phase estimation algorithm check                 */
/******************************************************************************/
 
#ifdef RAM_EXECUTE
void __ramfunc__ MCEST_ExecuteEstimator( void )
#else
void  MCEST_ExecuteEstimator( void )
#endif
{   
   
    uint16_t  u16a, u16b;
    int32_t   s32a, s32b, s32c;
   
    /* Read input signals */
    MCRPO_ReadInputSignals();  
    
    /* Estimate alpha-axis current */
    s32a = (int32_t)( gMCEST_InputSignal.ia - gMCEST_StateSignal.iaHat );
    s32b = (int32_t)( gMCEST_InputSignal.Ua - gMCEST_StateSignal.EaHat );
    s32c = (int16_t)( gMCEST_StateSignal.iaHat                         );

    s32a = MCLIB_SignumFunction( s32a );
    s32b = MCLIB_Multiplication( s32b, gMCEST_Parameters.h11 );
    s32c = MCLIB_Multiplication( s32c, gMCEST_Parameters.g11 );

    gMCEST_StateSignal.Za    = s32a;
    gMCEST_StateSignal.iaHat = s32a + s32b + s32c;


    /* Estimate beta-axis current */
    s32a = (int32_t)( gMCEST_InputSignal.ib - gMCEST_StateSignal.ibHat );
    s32b = (int32_t)( gMCEST_InputSignal.Ub - gMCEST_StateSignal.EbHat );
    s32c = (int32_t)( gMCEST_StateSignal.ibHat                         );

    s32a = MCLIB_SignumFunction( s32a );
    s32b = MCLIB_Multiplication( s32b, gMCEST_Parameters.h11 );
    s32c = MCLIB_Multiplication( s32c, gMCEST_Parameters.g11 );

    gMCEST_StateSignal.Zb    = s32a;
    gMCEST_StateSignal.ibHat = s32a + s32b + s32c;
      

    /* Estimate alpha-beta axis BEMF */
    mcLib_EulerFilter( &gMCEST_StateSignal.EaHat, gMCEST_StateSignal.Zb, gMCEST_Parameters.filtParam );
    mcLib_EulerFilter( &gMCEST_StateSignal.EbHat, gMCEST_StateSignal.Za, gMCEST_Parameters.filtParam );

    /* Estimate rotor speed and angle from estimated back EMF */
    u16a = mcLib_InverseTangent( gMCEST_StateSignal.EaHat, gMCEST_StateSignal.EbHat);
    u16a = u16a - PIHALVES;

    u16b = u16a - gMCEST_StateSignal.angle;
    s32a = (int16_t)(((uint32_t)u16b << SH_BASE_VALUE ) / K_SPEED_L);
    

    gMCEST_StateSignal.angle    = u16a;
    mcLib_EulerFilter( &gMCEST_StateSignal.speed, s32a, gMCEST_Parameters.filtParam);
 

    /* Write output signals */
    MCRPO_WriteOutputSignals();
   
       
}


/******************************************************************************/
/* Function name: MCRPO_ResetEstimator                                        */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* loosing of control of the phase estimation algorithm check                 */
/******************************************************************************/
void MCEST_ResetEstimator( void )
{
     
}