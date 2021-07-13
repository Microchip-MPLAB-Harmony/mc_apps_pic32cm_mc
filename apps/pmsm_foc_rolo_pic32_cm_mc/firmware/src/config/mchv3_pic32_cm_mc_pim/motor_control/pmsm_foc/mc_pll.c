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
#include "math.h"
#include "mc_pll.h"
#include "mc_math_library.h"
/*******************************************************************************
 * Interface Variables
*******************************************************************************/

/*******************************************************************************
 * Local/ Private Variables
*******************************************************************************/
static tmcRpo_InputSignal_s_   mcRpo_InputSignal_mds;
static tmcRpo_Parameters_s_    mcRpo_Parameters_mds;
static tmcRpo_StateSignal_s_   mcRpo_StateSignal_mds;
static tmcRpo_OutputSignal_s_ mcRpo_OutputSignal_mds;

/*******************************************************************************
 * Local/ Private Functions
*******************************************************************************/
void mcRpo_InportRead( void )
{
    
}


void mcRpo_OutportWrite( void )
{
    mcRpo_OutputSignal_mds.Phi = mcRpo_StateSignal_mds.Phi;
    mcRpo_OutputSignal_mds.We = mcRpo_StateSignal_mds.We;
}
/*******************************************************************************
 * Interface Functions
*******************************************************************************/
#define RPM_TO_ELEC_RAD_PER_SEC     ( 2.0f * M_PI  * NUM_POLE_PAIRS / ( 60.0f ))
#define VP_PER_KRPM_TO_VOLT_SEC_PER_RAD (float)( M_SQRT2 * 60.0f / ( 2000 * M_PI ) )

void mcRpoI_EstimatorInit( const tmcMoc_MotorParameters_s * const motorParam )
{
    float oneByKe;
    float Nt;
    
    /* Calculate back emf constant in terms of volt-sec/rad */
    oneByKe = 1.0f /( VP_PER_KRPM_TO_VOLT_SEC_PER_RAD * motorParam->Ke );
       
    /* Calculate transition speed in terms of electrical rad/s */
    Nt = 0.1f * VP_PER_KRPM_TO_VOLT_SEC_PER_RAD * motorParam->Nr;
    
    /* Calculate scaled parameters for PLL implementation */    
    mcRpo_Parameters_mds.Rs = (int16_t)((float)motorParam->Rs * BASE_VALUE / (float)mcRpo_Parameters_mds.baseImp );
    mcRpo_Parameters_mds.Ld = (int16_t)((float)motorParam->Ld * BASE_VALUE / (float)mcRpo_Parameters_mds.baseImp );
    mcRpo_Parameters_mds.Lq = (int16_t)((float)motorParam->Lq * BASE_VALUE / (float)mcRpo_Parameters_mds.baseImp );
    mcRpo_Parameters_mds.oneByKe = (int16_t)((float)oneByKe * BASE_VALUE / mcRpo_Parameters_mds.baseOneByKe  );
    mcRpo_Parameters_mds.Nmin =(int16_t)((float)Nt * BASE_VALUE / (float)mcRpo_Parameters_mds.baseSpe );
}

#define _multShift( a, b, sh )                  (((int32_t)a * (int32_t)b) >> sh ) 
#define _NegativeSign( a )   ( (int16_t)a & 0x8000)

void mcRpoI_EstimatorRun( void )
{
    int16_t Sine, Cosine;
    
    /* Read input port signals */
    mcRpo_InportRead();
    
    /* Calculate alpha axis back EMF */    
    mcRpo_StateSignal_mds.Ea = (int16_t)( mcRpo_InputSignal_mds.ua
                                                - ( _multShift( mcRpo_InputSignal_mds.ia, mcRpo_Parameters_mds.Rs, SH_BASE_VALUE )));
    
    /* Calculate beta axis back EMF */
    mcRpo_StateSignal_mds.Eb = (int16_t)( mcRpo_InputSignal_mds.ub
                                                - ( _multShift( mcRpo_InputSignal_mds.ib, mcRpo_Parameters_mds.Rs, SH_BASE_VALUE )));
    
    /* Calculate direct-quadrature axis back EMF  */
    Sine = mcLib_Sine( mcRpo_StateSignal_mds.Phi );
    Cosine = mcLib_Cosine( mcRpo_StateSignal_mds.Phi );
    
    mcRpo_StateSignal_mds.Ed = _multShift( Cosine, mcRpo_StateSignal_mds.Ea,  SH_BASE_VALUE ) 
                                                + _multShift( Sine, mcRpo_StateSignal_mds.Eb,  SH_BASE_VALUE );
    
    mcRpo_StateSignal_mds.Eq = _multShift( -Sine, mcRpo_StateSignal_mds.Ea,  SH_BASE_VALUE ) 
                                                + _multShift( Cosine, mcRpo_StateSignal_mds.Eb,  SH_BASE_VALUE );
    
    /* Estimate rotor speed using PLL logic */
    if( _NegativeSign( mcRpo_StateSignal_mds.Eq ))
    {
        int32_t s32a;
        s32a = mcRpo_StateSignal_mds.Eq + mcRpo_StateSignal_mds.Ed;
        mcRpo_StateSignal_mds.We = _multShift( mcRpo_Parameters_mds.oneByKe, s32a, SH_BASE_VALUE );
    }
    else 
    {
        int32_t s32a;
        s32a = mcRpo_StateSignal_mds.Eq - mcRpo_StateSignal_mds.Ed;
        mcRpo_StateSignal_mds.We = _multShift( mcRpo_Parameters_mds.oneByKe, s32a, SH_BASE_VALUE );    
    }
    
    /* Integrate the rotor speed to determine phase angle */
    mcRpo_StateSignal_mds.Phi += _multShift( mcRpo_StateSignal_mds.We, mcRpo_Parameters_mds.Ts, SH_BASE_VALUE );
    
    /* Update output ports */
    mcRpo_OutportWrite( );
    
}

void mcRpoI_BaseParametersSet( float baseCur, float baseVol, float baseSpe )
{
    mcRpo_Parameters_mds.baseCur  = baseCur;
    mcRpo_Parameters_mds.baseVol  = baseVol;
    mcRpo_Parameters_mds.baseSpe = baseSpe * RPM_TO_ELEC_RAD_PER_SEC;
    mcRpo_Parameters_mds.baseImp = baseVol / baseCur;
    mcRpo_Parameters_mds.baseOneByKe =  baseSpe / baseVol;
}

void mcRpoI_EstimatorReset( void )
{
    
}
/* *****************************************************************************
 End of File
 */
