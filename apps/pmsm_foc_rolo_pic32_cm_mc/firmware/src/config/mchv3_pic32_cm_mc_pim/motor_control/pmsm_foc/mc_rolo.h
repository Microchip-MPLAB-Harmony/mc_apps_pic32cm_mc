/*******************************************************************************
  System Definitions

  File Name:
    mc_rolo.h

  Summary:
    Header file which contains variables and function prototypes for angle and speed estimation
    with reduced order Luenberger observer 
 
  Description:
    This file contains variables and function prototypes which are generally used for angle and 
    speed estimation with reduced order Luenberger observer .  Implemented in Q2.14 Fixed
     Point Arithmetic.
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

#ifndef Q14_ROLO_MCLIB_H
#define Q14_ROLO_MCLIB_H

#include <stdint.h>
#include <sys/attribs.h>
#include "mc_math_library.h"
#include "mc_userparams.h"

/*******************************************************************************
Macro definitions
*******************************************************************************/
#define	OBS_H_GAIN			( 0.2f )				/* RANGE: 0.2 - 0.5 */
#define	OBS_C0_GAIN			( 1.0f - OBS_H_GAIN )	/* > 0 */
#define	OBS_MINFREQ_HZ		( 3.0f )
#define	OBS_MINSPEED_R_S	((float)(2.0f * FLOAT_PI * OBS_MINFREQ_HZ))
#define OBS_MAXSHIFTS		( 32 )



/* uncomment the following macro to enable phase clamping to enhance waveform */
/* #define PH_CLAMP */
#define MAX_DSPEED			( 300 )

/*	only the cross-coupling coefficients are speed dependent, and their value is usually
	much lesser than the others; so in first approximation they could be neglected, saving
	a lot of computation time; furthermore, neglecting them could lead to a higher noise immunity */
#define CROSS_COUPLING_ENABLED 

/* uncomment the following macro to enable amplification clamping */
/* #define AMP_CLAMP */

/*******************************************************************************
Type definitions
*******************************************************************************/
typedef struct _tmcRpo_InputSignal_s
{
    int16_t  ia;		// measured alpha current 
    int16_t  ib;		// measured beta current
    int16_t  ua;		// applied alpha voltage
    int16_t  ub;		// applied beta voltage   
}tmcRpo_InputSignal_s;

typedef struct _tmcRpo_Parameters_s
{
   float f32_sam_fre; /* sampling frequency [Hz] */
   float f32_bas_spe;/* base speed [rad/sec] */
   float f32_k_volam; /* internal conversion constant */ 
   float f32_sta_res;/* stator resistance [Ohm] */
   float f32_syn_ind; /* synchronous inductance [Hen] */
   float f32_k_gain;  /* constant in observer coefficient calculation */
   float f32_c1_coe;  /* constant in observer coefficient calculation */
#ifdef  CROSS_COUPLING_ENABLED
   float f32_k_speed; /* internal conversion constant */
#endif
}tmcRpo_Paramaters_s;

typedef struct _tmcRpo_ObserverCoefficinets_s
{
  IQ l11;   /* constant observer coefficient */
  IQ m11;   /* constant observer coefficient */
  IQ n11;  /* constant observer coefficient */
  IQ k11;   /* constant observer coefficient */
#ifdef  CROSS_COUPLING_ENABLED
  IQ hhh;   /* intermediate result in observer variable coefficient calculation */
  IQ lll;   /* intermediate result in observer variable coefficient calculation */
  IQ xxx;   /* intermediate result in observer variable coefficient calculation */
  IQ m21;   /* observer variable coefficient */
  IQ n21;   /* observer variable coefficient */
  IQ k21;   /* observer variable coefficient */
#endif  // ifdef CROSS_COUPLING_ENABLED
    
}tmcRpo_ObserverCoefficients_s;

typedef struct _tmcRpo_StateSignal_s
{
   tmcLib_Cartesian_s   obs_z;   /* observer estimated status vector */
   tmcLib_Cartesian_s   obs_e;   /* observer estimated  bemf vector */
   tmcLib_Polar_s  bemf;               /* observer estimated  bemf vector (polar coordinates) */
   int32_t  sp_fir_acc;  /* speed fir filter accumulator */
   int32_t  sp_iir1_mem; /* speed iir filter memory (first iir) */
   int32_t  sp_iir2_mem; /* speed iir filter memory (second iir) */
   int32_t  sp_iir3_mem; /* speed iir filter memory (third iir) */
   uint16_t  dph_min;     /* minimum delta phase */
         
#ifdef PH_CLAMP
  uint16_t  max_dspeed;  /* maximum speed variation in one sampling period */
#endif // ifdef PH_CLAMP
   uint16_t  flx_arg;  /* estimated permanent magnets flux position */
   uint16_t  flx_arg_mem; /* estimated permanent magnets flux position memory */
   uint16_t  dph_abs_fil; /* filtered one-step phase difference */

   uint16_t  dph_global;
   int16_t  k_spe12;  /* internal conversion constant */
   int16_t  speed_min;  /* minimum speed [internal units] */
   int16_t  speed_est;  /* estimated speed [internal units] */
   int16_t  speed_sgn;  /* speed sign */
   int16_t  speed_abs;  /* speed absolute value */
   int16_t  sp_fir_ind;  /* index in fir speed filter memories vector */
   int16_t  sp_fir_vec[8]; /* speed fir filter memories vector */
   
   int16_t  ua;
   int16_t  ub;

}tmcRpo_StateSignal_s;

typedef struct _tmcRpo_OutputSignal_s
{
    uint16_t angle;
    int16_t speed;
    int16_t Es;
    
}tmcRpo_OutputSignal_s;

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
void mcRpoI_InitializeObserver( void );

/* !\brief Execute Luenberger Observer 
 * 
 * Details.
 * This interface functions reads phase current and voltage space vector inputs ( alpha -beta ),
 *  and estimates the rotor phase angle and speed
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return: None
 */
#ifdef RAM_EXECUTE
void __ramfunc__ MCRPO_ExecuteEstimator( void );
#else
void MCRPO_ExecuteEstimator( void );
#endif

/* !\brief Returns angular position
 * 
 * Details.
 * This interface functions returns the scaled angular position of the motor 
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
#ifdef RAM_EXECUTE
uint16_t __ramfunc__ mcRpo_GetAngularPosition(void);
#else
uint16_t mcRpo_GetAngularPosition(void);
#endif

/* !\brief Returns angular speed
 * 
 * Details.
 * This interface functions returns the scaled angular speed of the motor 
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular speed 
 */
#ifdef RAM_EXECUTE
int16_t __ramfunc__ mcRpo_GetAngularSpeed(void);
#else
int16_t mcRpo_GetAngularSpeed(void);
#endif

/* !\brief Reset Observer  
 * 
 * Details.
 * This interface functions reset the observer
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return: None
 */
void mcRpo_ResetObserver( void );

#endif // Q14_ROLO_MCLIB_H
