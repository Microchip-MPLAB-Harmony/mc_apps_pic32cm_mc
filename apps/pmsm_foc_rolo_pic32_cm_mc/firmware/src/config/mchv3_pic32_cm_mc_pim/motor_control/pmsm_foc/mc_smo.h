/*******************************************************************************
  System Definitions

  File Name:
    mc_infrastructure.h

  Summary:
    Header file which contains variables and function prototypes to coordinate
    motor Control.

  Description:
    This file contains variables and function prototypes which are generally
    used in to coordinate motor control functions. It is implemented in Q2.14
    fixed Point Arithmetic.

 *******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef MC_SMO_H
#define MC_SMO_H

#include <stdint.h>
#include <sys/attribs.h>
#include <math.h>
#include "userparams.h"


/*******************************************************************************/
/*                      USER DEFINED PARAMETERS                                */
/*******************************************************************************/

#define BEMF_FILTER_COEFF                                     (int16_t)( 1200                               )
#define BEMF_FILTER_SHIFT                                     (uint16_t)( SH_BASE_VALUE                     )
#define SPEED_FILTER_COEFF                                    (int16_t)( 500                                )
#define SPEED_FILTER_SHIFT                                    (uint16_t)( SH_BASE_VALUE                     )
#define PMSM_TIME_CONSTANT_IN_SEC                             (float)( MOTOR_PER_PHASE_RESISTANCE_IN_OHM / MOTOR_PER_PHASE_INDUCTANCE_IN_HENRY )
#define EXPONENT_MINUS_RSTSDIVLS                              (float)( exp( -PMSM_TIME_CONSTANT_IN_SEC / (float)CURRENT_CONTROL_FREQUENCY ))
#define SLIDING_MODE_GAIN                                     (float)( 6000.0f)
#define CURRENT_OBSERVER_BOUNDARY_LAYER_IN_AMPERE             (float)( 0.50f)
#define CURRENT_OBSERVER_BOUNDARY_LAYER_IN_INTERNAL_UNIT      (int16_t)( K_CURRENT * CURRENT_OBSERVER_BOUNDARY_LAYER_IN_AMPERE )


/* ____________________________________CURRENT OBSERVER COEFFICIENTS ______________________________*/

/* SYSTEM MATRIX */
#define CURRENT_OBSERVER_SYSTEM_MATRIX_G11                    (float)( EXPONENT_MINUS_RSTSDIVLS )
#define CURRENT_OBSERVER_SYSTEM_MATRIX_G12                    (float)( 0.0f )
#define CURRENT_OBSERVER_SYSTEM_MATRIX_G21                    (float)( 0.0f )
#define CURRENT_OBSERVER_SYSTEM_MATRIX_G22                    (float)( EXPONENT_MINUS_RSTSDIVLS  )

/* INPUT PARAMETERS  */
#define CURRENT_OBSERVER_INPUT_MATRIX_H11                     (float)( ( 1.0f - EXPONENT_MINUS_RSTSDIVLS ) / ( MOTOR_PER_PHASE_RESISTANCE_IN_OHM  ))
#define CURRENT_OBSERVER_INPUT_MATRIX_H12                     (float)( ( 0.0f )                                                                )
#define CURRENT_OBSERVER_INPUT_MATRIX_H21                     (float)( ( 0.0f )                                                                )
#define CURRENT_OBSERVER_INPUT_MATRIX_H22                     (float)( ( 1.0f - EXPONENT_MINUS_RSTSDIVLS ) / ( MOTOR_PER_PHASE_RESISTANCE_IN_OHM ))

/* SLIDING ACTION MATRIX */
#define CURRENT_OBSERVER_SLIDE_MATRIX_B11                     (float)( SLIDING_MODE_GAIN * ( 1.0f - EXPONENT_MINUS_RSTSDIVLS ) / ( PMSM_TIME_CONSTANT_IN_SEC  ))
#define CURRENT_OBSERVER_SLIDE_MATRIX_B12                     (float)( ( 0.0f )                                                        )                 
#define CURRENT_OBSERVER_SLIDE_MATRIX_B21                     (float)( ( 0.0f )                                                        )  
#define CURRENT_OBSERVER_SLIDE_MATRIX_B22                     (float)( SLIDING_MODE_GAIN * ( 1.0f - EXPONENT_MINUS_RSTSDIVLS ) / ( PMSM_TIME_CONSTANT_IN_SEC ))

/*******************************************************************************/
/*                   USER DEFINED DATA STRUCTURES                              */
/*******************************************************************************/
typedef struct
{
  int16_t                ia;                 /*    Alpha axis current              */
  int16_t                ib;                 /*    Beta axis current               */
  int16_t                Ua;                 /*    Alpha axis voltage              */
  int16_t                Ub;                 /*    Beta axis voltage               */
}tMCEST_INPUT_SIGNAL_S;

typedef struct
{
    IQ                   g11;               /*    System matrix element G[1][1]    */
    IQ                   g22;               /*    System matrix element G[2][2]    */
    IQ                   h11;               /*    Input matrix element  H[1][1]    */
    IQ                   h22;               /*    Input matrix element  H[2][2]    */
    IQ                   b11;               /*    Slide matrix element  B[1][1]    */
    IQ                   b22;               /*    Slide matrix element  B[2][2]    */
    int16_t              m;                 /*    Sliding action                   */
    int16_t              mdbi;
    int16_t              boundary; 
    IQ                   filtParam;         /*    Filter parameter                 */

}tMCEST_PARAMETERS_S;

typedef struct
{
    int16_t              iaHat;             /*    Alpha axis current               */
    int16_t              ibHat;             /*    Beta axis current                */
    int16_t              EaHat;             /*    estimated speed                  */
    int16_t              EbHat;             /*    PLL estimator gain               */
    int16_t              Za;
    int16_t              Zb;
    uint16_t             angle;             /*    Flux angle                       */
    int16_t              speed;             /*    Electrical speed of PMSM         */
}tMCEST_STATE_SIGNAL_S;

typedef struct
{
   uint16_t              angle;             /*    Estimated output                */
   int16_t               speed;             /*    Estimated speed                 */

}tMCEST_OUTPUT_SIGNAL_S;

/*******************************************************************************/
/*                         INTERFACE VARIABLES                                 */
/*******************************************************************************/

extern tMCEST_INPUT_SIGNAL_S  gMCEST_InputSignal;
extern tMCEST_STATE_SIGNAL_S  gMCEST_StateSignal;
extern tMCEST_OUTPUT_SIGNAL_S gMCEST_OutputSignal;

/*******************************************************************************/
/*                         LOCAL FUNCTIONS                                     */
/*******************************************************************************/

/*******************************************************************************/
/*                         INTERFACE FUNCTIONS                                 */
/*******************************************************************************/


/******************************************************************************/
/* Function name: MCRPO_InitializeEstimator                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* loosing of control of the phase estimation algorithm check                 */
/******************************************************************************/
void MCEST_InitializeEstimator( void );

/******************************************************************************/
/* Function name: MCRPO_ExecuteEstimator                                      */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* loosing of control of the phase estimation algorithm check                 */
/******************************************************************************/
#ifdef RAM_EXECUTE
void __ramfunc__ MCEST_ExecuteEstimator(void );
#else
void  MCEST_ExecuteEstimator( void );
#endif

/******************************************************************************/
/* Function name: MCRPO_ResetEstimator                                        */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* loosing of control of the phase estimation algorithm check                 */
/******************************************************************************/
void MCEST_ResetEstimator( void );

#endif // MC_SMO_H
