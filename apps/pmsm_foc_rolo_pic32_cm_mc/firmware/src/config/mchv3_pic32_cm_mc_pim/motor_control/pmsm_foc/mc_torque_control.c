/*******************************************************************************
 Motor Control Application Source file 

  Company:
    Microchip Technology Inc.

  File Name:
    mc_torque_control.c

  Summary:
    This file contains all the functions related to torque control 

  Description:
    This file contains implementation of torque control functions
 
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
#include <stdlib.h>
#include "definitions.h"
#include <sys/attribs.h>
#include "mc_rolo.h"
#include "mc_pwm.h"
#include "mc_math_library.h"
#include "mc_generic_library.h"
#include "mc_torque_control.h"
#include "mc_function_coordinator.h"
#include "mc_current_measurement.h"
#include "mc_voltage_measurement.h"
#include "mc_hardware_abstraction.h"
#include "mc_motor_control.h"

/*******************************************************************************
 * Global variables 
*******************************************************************************/
int16_t mcTorI_QaxisReferenceCurrent_gds16 ;
int16_t mcTorI_DaxisReferenceCurrent_gds16 ;

/*******************************************************************************
 * Module variables 
*******************************************************************************/

static tmcTor_InputSignal_s    mcTor_InputSignal_mds;
static tmcTor_Parameters_s    mcTor_Parameters_mds;
static tmcTor_OutputSignal_s  mcTor_OutputSignal_mds;

/*******************************************************************************
 * Local Functions  
*******************************************************************************/
__STATIC_INLINE void mcTor_ReadInputSignal( void )
{
#ifdef TORQUE_MODE  
    if( RUNNING == mcMocI_MotorControlState_gde )
    {
    #ifdef TORQUE_INPUT_FROM_POTENTIOMETER
        mcTor_InputSignal_mds.potInput  = mcHal_PotAdcValue_gdu16;
        mcTorI_QaxisReferenceCurrent_gds16 = (  mcTor_InputSignal_mds.potInput 
                                                                         * mcTor_Parameters_mds.currentForMaxTorque ) >> 12;   
    #endif
    #ifdef BIDIRECTION_CONTROL
        mcTorI_QaxisReferenceCurrent_gds16 *= mcMocI_DirectionSign_gds16;
        if( mcTor_Parameters_mds.currentForMaxTorque <  abs(mcTorI_QaxisReferenceCurrent_gds16 ))
        {
             mcTorI_QaxisReferenceCurrent_gds16 = mcMocI_DirectionSign_gds16 * mcTor_Parameters_mds.currentForMaxTorque;
        }
        else if( mcTor_Parameters_mds.currentForMinTorque > abs( mcTorI_QaxisReferenceCurrent_gds16 ))
        {
             mcTorI_QaxisReferenceCurrent_gds16 = mcMocI_DirectionSign_gds16 * mcTor_Parameters_mds.currentForMinTorque;
        }  
        else 
        {
            /*Do nothing */
        }
    #else
        if( mcTor_Parameters_mds.currentForMaxTorque <  mcTorI_QaxisReferenceCurrent_gds16 )
        {
             mcTorI_QaxisReferenceCurrent_gds16 = mcTor_Parameters_mds.currentForMaxTorque;
        }
        else if( mcTor_Parameters_mds.currentForMinTorque >  mcTorI_QaxisReferenceCurrent_gds16 )
        {
             mcTorI_QaxisReferenceCurrent_gds16 = mcTor_Parameters_mds.currentForMinTorque;
        }
        else 
        {
            /*Do nothing */
        }
    #endif
    }
    
#endif 
    
    mcTor_InputSignal_mds.idRef = mcTorI_DaxisReferenceCurrent_gds16;
    mcTor_InputSignal_mds.iqRef = mcTorI_QaxisReferenceCurrent_gds16;
       
    mcTor_InputSignal_mds.idAct = mcCur_MeasuredDaxisCurrent_gds16;
    mcTor_InputSignal_mds.iqAct = mcCur_MeasuredQaxisCurrent_gds16;
    
    mcTor_InputSignal_mds.UacPeak = mcVolI_InverterAcVoltagePeak_gds16; 
}

__STATIC_INLINE void mcTor_WriteOutputSignal( void )
{
    
}

/*******************************************************************************
Interface Functions
*******************************************************************************/

/*! \brief Initialize PMSM torque control 
 * 
 * Details.
 * Initialize PMSM torque control 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcTorI_InitializeTorqueControl( void )
{    
#ifdef ISOTROPIC_MOTOR
    mcTor_Parameters_mds.Psi                           =    AIR_GAP_FLUX;
    mcTor_Parameters_mds.Ld                            =    DIRECT_AXIS_INDUCTANCE_IN_HENRY;
    mcTor_Parameters_mds.Lq                            =    QUADRATURE_AXIS_INDUCTANCE_IN_HENRY;
    mcTor_Parameters_mds.LdMinusLq                =    mcTor_Parameters_mds.Ld  - mcTor_Parameters_mds.Lq;
    mcTor_Parameters_mds.TwoByThreePz         =    2.0f / ( 3.0f * NUM_POLE_PAIRS );
    mcTor_Parameters_mds.PsiBy2LdMinusLq      =    0.5 * mcTor_Parameters_mds.Psi / mcTor_Parameters_mds.LdMinusLq;
    mcTor_Parameters_mds.PsiBy2LdMinusLqSqr =   mcTor_Parameters_mds.PsiBy2LdMinusLq
                                                                          *   mcTor_Parameters_mds.PsiBy2LdMinusLq;
#else 
    mcTor_Parameters_mds.Psi =  (float)AIR_GAP_FLUX;
    mcTor_Parameters_mds.ScaledCurrentToTorque =  0.5f * mcTor_Parameters_mds.Psi/K_CURRENT;
    mcTor_Parameters_mds.TorqueToScaledCurrent =  1.0f / mcTor_Parameters_mds.ScaledCurrentToTorque;
#endif
    
    /* D axis current PI controller initialization */
    mcLib_PiControllerParametersSet( D_CURRENT_KP, D_CURRENT_KI, D_CURRENT_YMAX,
                                                        &mcTor_Parameters_mds.idState );
    
    /* Q axis current PI controller initialization */
    mcLib_PiControllerParametersSet( Q_CURRENT_KP, Q_CURRENT_KI,  Q_CURRENT_YMAX, 
                                                          &mcTor_Parameters_mds.iqState );
    
}


/*! \brief Execute PMSM torque control 
 * 
 * Details.
 * Set PMSM torque
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
#ifdef RAM_EXECUTE
void  __ramfunc__ mcTorI_ExecuteTorqueControl(void)
#else
void mcTorI_ExecuteTorqueControl( void )
#endif
{
     tmcLib_PiControllerState_s  * idState;
     tmcLib_PiControllerState_s  * iqState; 

     /* Read input signals */
     mcTor_ReadInputSignal();
     
     idState = &mcTor_Parameters_mds.idState;
     iqState = &mcTor_Parameters_mds.iqState;
     
     if( RUNNING == mcMocI_MotorControlState_gde )
     {
        /* Execute D axis current control */
        idState->Ymax = (int16_t)mcTor_InputSignal_mds.UacPeak;      
        idState->Ymin = - idState->Ymax;
               
        idState->input = mcTor_InputSignal_mds.idRef;
        idState->feedback = mcTor_InputSignal_mds.idAct;
        mcTor_OutputSignal_mds.Ud = mcLib_PiControllerAutoModeRun( idState );
        
     }
     
     /* Execute Q axis current Control */ 
     iqState->Ymax =   mcLib_DetermineAdjSide(mcTor_InputSignal_mds.UacPeak, mcTor_OutputSignal_mds.Ud); 
     iqState->Ymin  = - iqState->Ymax;
     iqState->input = mcTor_InputSignal_mds.iqRef;
     iqState->feedback = mcTor_InputSignal_mds.iqAct; 
     mcTor_OutputSignal_mds.Uq = mcLib_PiControllerAutoModeRun( iqState );
  
     		      
     /* Execute SVPWM Modulation */
     mcPwmI_DaxisVoltage_gds16 = mcTor_OutputSignal_mds.Ud;
     mcPwmI_QaxisVoltage_gds16 =  mcTor_OutputSignal_mds.Uq;
     mcPwm_PwmModulation();
     
     /* Write output signals */
     mcTor_WriteOutputSignal();
}

/*! \brief Set PMSM  torque
 * 
 * Details.
 * Set PMSM torque
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcTorI_MotorTorqueSet( float f_Torque_df32 )
{
#ifndef NON_ISOTROPIC
    /* Calculate q-axis reference current for commanded torque */
    mcTorI_QaxisReferenceCurrent_gds16 = mcTor_Parameters_mds.TorqueToScaledCurrent * f_Torque_df32;
    mcTorI_DaxisReferenceCurrent_gds16 = 0.0f;
    
#else
    int32_t Numerator, Denominator;
    Numerator = (int32_t)( f_Torque_df32 * K_CURRENT );
    Numerator = ( mcTor_Parameters_mds.TwoByThreePz * Numerator ) << SH_BASE_VALUE;
    
    Denominator = ( mcTorI_DaxisReferenceCurrent_gds16 * mcTor_Parameters_mds.LdMinusLq ) >> SH_BASE_VALUE;
    Denominator = Denominator +   mcTor_Parameters_mds.Psi;
    
    mcTorI_QaxisReferenceCurrent_gds16 = Numerator/ Denominator;
    
    /* Determine d-axis current */
    mcFlx_ExecuteFluxControl( );
    
#endif
    
 }


/*! \brief Get PMSM  torque
 * 
 * Details.
 * Get PMSM torque
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */

float mcTorI_MotorTorqueGet( void  )
{
    float Torque = 0.0f;
     
    /* Calculate q-axis reference current for commanded torque */
#ifndef NON_ISOTROPIC
    Torque = mcTor_Parameters_mds.ScaledCurrentToTorque * (float)mcTorI_QaxisReferenceCurrent_gds16;
#else
    /* ToDo: Derive proper distribution of current in D and Q axis */
#endif
   
    return Torque;
}

/*! \brief Set maximum torque 
 * 
 * Details.
 * Set maximum torque 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcTorI_MaximumTorqueSet( float f_maxTorque_df32 )
{
    mcTor_Parameters_mds.currentForMaxTorque = (int16_t)(   f_maxTorque_df32 
                                                                                               * mcTor_Parameters_mds.TorqueToScaledCurrent );
}

/*! \brief Set minimum torque 
 * 
 * Details.
 * Set minimum torque 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcTorI_MinimumTorqueSet( float f_minTorque_df32 )
{
    mcTor_Parameters_mds.currentForMinTorque = (int16_t)(   f_minTorque_df32 
                                                                                              * mcTor_Parameters_mds.TorqueToScaledCurrent );
}


/* EOF motor_control.c */

