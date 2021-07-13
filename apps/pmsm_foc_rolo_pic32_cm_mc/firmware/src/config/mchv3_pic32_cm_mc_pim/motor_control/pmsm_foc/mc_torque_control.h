/*******************************************************************************
  System Definitions

  File Name:
    mc_torque_control.h

  Summary:
    Header file which shares global variables and function prototypes.

  Description:
    This file contains the global variables and function prototypes for motor torque control.
    Implemented in Q2.14 format..

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

#ifndef MC_TOR_H
#define MC_TOR_H

/*******************************************************************************
Headers inclusions
*******************************************************************************/
#include <stdint.h>
#include <sys/attribs.h>
#include "mc_generic_library.h"
#include "math.h"
#include "mc_userparams.h"


/*******************************************************************************
 * Configuration parameters
*******************************************************************************/
/**
 * Scaled current to Torque conversion factor
 * Torque [ Nm] = TORQUE_PER_AMPERE_SCALED_CURRENT * Quadrature Current [ Internal Units ]
 */
#define  TORQUE_PER_AMPERE_SCALED_CURRENT     (float)( 1.5 * AIR_GAP_FLUX / K_CURRENT )

/**
 * Torque to scaled current conversion factor 
 * Quadrature Current [ Internal Units ] = SCALED_CURRENT_PER_NEWTON_METRE * Torque [ Nm ]
 */
#define  SCALED_CURRENT_PER_NEWTON_METRE      (float)( 1.0f / TORQUE_PER_AMPERE_SCALED_CURRENT )


/*******************************************************************************
 * Use-defined data structure 
*******************************************************************************/

typedef struct _tmcTor_InputSignal_s
{
#ifdef TORQUE_INPUT_FROM_POTENTIOMETER
    int16_t              potInput;
#endif
    int16_t              idRef;
    int16_t              idAct;
    int16_t              iqRef;
    int16_t              iqAct;
    int16_t              UacPeak;
       
}tmcTor_InputSignal_s;

typedef struct _tmcTor_Parameters_s
{
    int16_t                           currentForMinTorque;
    int16_t                           currentForMaxTorque;
    tmcLib_PiControllerState_s  iqState;
    tmcLib_PiControllerState_s  idState;   
    float                               Psi;
#ifdef ISOTROPIC_MOTOR
    float                               TwoByThreePz;
    float                               LdMinusLq;
    float                               PsiBy2LdMinusLq;
    float                               PsiBy2LdMinusLqSqr;
#else 
    float                             ScaledCurrentToTorque;
    float                             TorqueToScaledCurrent;
#endif
}tmcTor_Parameters_s;


typedef struct _tmcTor_OutputSignal_s
{
    int16_t                Ud;
    int16_t                Uq;
}tmcTor_OutputSignal_s;


/*******************************************************************************
 * Interface variables 
*******************************************************************************/
extern int16_t mcTorI_QaxisReferenceCurrent_gds16 ;
extern int16_t mcTorI_DaxisReferenceCurrent_gds16 ;

/*******************************************************************************
 * Interface Functions
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
void mcTorI_InitializeTorqueControl( void );


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
void  __ramfunc__ mcTorI_ExecuteTorqueControl(void);
#else
void mcTorI_ExecuteTorqueControl( void );
#endif

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
void mcTorI_MotorTorqueSet( float f_Torque_df32 );


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

float mcTorI_MotorTorqueGet( void  );


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
void mcTorI_MaximumTorqueSet( float f_maxTorque_df32 );


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
void mcTorI_MinimumTorqueSet( float f_minTorque_df32 );

#endif // MC_SUP_H