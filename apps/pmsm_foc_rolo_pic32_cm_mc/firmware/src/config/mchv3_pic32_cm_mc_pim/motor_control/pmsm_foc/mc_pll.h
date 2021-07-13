/*******************************************************************************
  System Definitions

  File Name:
    mc_generic_library.h

  Summary:
    Header file which contains variables and function prototypes of  generic library functions.
 
  Description:
    This file contains variables and function prototypes of generic library functions 
    which are generally used in Motor Control. Implemented in Q2.14 Fixed Point Arithmetic.
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

#ifndef MC_PLL_H
#define MC_PLL_H

#include <stdint.h>
#include <sys/attribs.h>
#include "userparams.h"


/******************************************************************************
 * Constants 
******************************************************************************/

/******************************************************************************
 * User-defined data-structure 
******************************************************************************/
typedef struct
{
    int16_t ua;
    int16_t ub;
    int16_t ia;
    int16_t ib;
}tmcRpo_InputSignal_s_;

typedef struct 
{
    int16_t      Rs;           //   Motor per phase resistance in internal units               
    int16_t      Ld;           //   Motor direct axis inductance in internal units       
    int16_t      Lq;           //   Motor quadrature axis inductance in internal units 
    int16_t      oneByKe;           //   Motor back emf constant in internal units  
    int16_t      Nmin;       //   Minimum speed for rotor position calculation in internal units
    int16_t      Ts;
    float       baseCur;
    float       baseVol;
    float       baseSpe;
    float       baseImp;
    float       baseOneByKe;
}tmcRpo_Parameters_s_;

typedef struct 
{
    int16_t Ea;
    int16_t Eb;
    int16_t Ed;
    int16_t Eq;
    int16_t We;
    int16_t Phi;    
}tmcRpo_StateSignal_s_;


typedef struct 
{
    uint16_t Phi;
    int16_t  We;  
}tmcRpo_OutputSignal_s_;

typedef struct 
{
    float      Rs;            //  Motor per phase resistance in Ohm                
    float      Ld;           //   Motor direct axis inductance in Henry          
    float      Lq;           //   Motor quadrature axis inductance in Henry   
    float      Ke;          //   Motor back emf constant in V-peak/ kRPM    
    float      Nr;          //   Motor rated speed    
}tmcMoc_MotorParameters_s;

/*******************************************************************************
 * Interface variables 
*******************************************************************************/
extern uint16_t mcRpoI_ElectricalRotorPosition_gdu16;
extern int16_t  mcRpoI_ElectricalRotorSpeed_gds16;
extern int16_t  mcRpoI_BacKEmfMagnitude_gds16;

/*******************************************************************************
 * Interface functions
*******************************************************************************/

/* !\brief Initialize Observer  
 * 
 * Details.
 * This interface functions initializes the observer
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return: None
 */
void mcRpoI_EstimatorInit( const tmcMoc_MotorParameters_s * const motorParam );

/* !\brief Execute PLL Estimator 
 * 
 * Details.
 * This interface functions reads phase current and voltage space vector inputs ( alpha -beta ),
 *  and estimates the rotor phase angle and speed using PLL logic
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return: None
 */
void mcRpoI_EstimatorRun( void );

/* !\brief Estimator Reset 
 * 
 * Details.
 * This interface functions returns the scaled angular position of the motor 
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
void mcRpoI_EstimatorReset(void);

/* !\brief Estimator Reset 
 * 
 * Details.
 * This interface functions returns the scaled angular position of the motor 
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
void mcRpoI_BaseParametersSet( float baseCur, float baseVol, float baseSpe );

#endif //MC_PLL_H
