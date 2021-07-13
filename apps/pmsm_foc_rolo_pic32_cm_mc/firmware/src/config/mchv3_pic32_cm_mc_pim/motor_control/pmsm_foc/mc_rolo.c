/*******************************************************************************
 Reduced Order Luenberger Observer Motor Control Library 

  Company:
    Microchip Technology Inc.

  File Name:
    mc_rolo.c

  Summary:
    Reduced Order Luenberger Observer related functions and variables
    implemented in Q14 fixed point arithmetic.

  Description:
    This file implements reduced order luenberger observer related functions
 
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
#include "mc_rolo.h"
#include "definitions.h"
#include "mc_userparams.h"
#include "mc_motor_control.h"
#include "mc_pwm.h"
#include "mc_current_measurement.h"


/*******************************************************************************
Private global variables
*******************************************************************************/
static tmcRpo_InputSignal_s                  mcRpo_InputSignal_mds;
static tmcRpo_Paramaters_s                   mcRpo_Parameters_mds;
static tmcRpo_ObserverCoefficients_s   mcRpo_ObserverCoefficients_mds;
static tmcRpo_StateSignal_s                  mcRpo_StateSignal_mds;
static tmcRpo_OutputSignal_s                mcRpo_OutputSignal_mds;


/******************************************************************************
Safety variables
******************************************************************************/
uint16_t mcRpoI_ElectricalRotorPosition_gdu16;
int16_t  mcRpoI_ElectricalRotorSpeed_gds16;
int16_t  mcRpoI_BacKEmfMagnitude_gds16;
 
/*******************************************************************************
Functions (private and public)
*******************************************************************************/

/* !\brief Initialize Speed Filter 
 * 
 * Details.
 * Initialize Speed Filter 
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
void mcRpo_SpeedFilterInit(int16_t speed)
{
    int16_t temp;

    mcRpo_StateSignal_mds.speed_est = speed;
    if(0 > mcRpo_StateSignal_mds.speed_est)
    {
        mcRpo_StateSignal_mds.speed_abs = -mcRpo_StateSignal_mds.speed_est;
        mcRpo_StateSignal_mds.speed_sgn = -1;
    }
    else
    {
        mcRpo_StateSignal_mds.speed_abs = mcRpo_StateSignal_mds.speed_est;
        mcRpo_StateSignal_mds.speed_sgn = 1;
    }
    mcRpo_StateSignal_mds.sp_iir3_mem = ((int32_t)mcRpo_StateSignal_mds.speed_abs) * ((int32_t)mcRpo_StateSignal_mds.k_spe12);
    mcRpo_StateSignal_mds.sp_iir2_mem = mcRpo_StateSignal_mds.sp_iir3_mem >> 4;
    mcRpo_StateSignal_mds.sp_iir1_mem = mcRpo_StateSignal_mds.sp_iir2_mem >> 4;
    mcRpo_StateSignal_mds.sp_fir_acc = mcRpo_StateSignal_mds.sp_iir1_mem >> 4;

    temp = (int16_t)(mcRpo_StateSignal_mds.sp_fir_acc >> 2);  /*speed calculated over 4 samples */
    
    /* temp = (int16_t)(mcRpo_StateSignal_mds.sp_fir_acc >> 3); speed calculated over 8 samples */
    mcRpo_StateSignal_mds.dph_abs_fil = (uint16_t)(temp);

    for(mcRpo_StateSignal_mds.sp_fir_ind = 0; mcRpo_StateSignal_mds.sp_fir_ind < 8; mcRpo_StateSignal_mds.sp_fir_ind++)
    {
       mcRpo_StateSignal_mds.sp_fir_vec[mcRpo_StateSignal_mds.sp_fir_ind] = (int16_t)mcRpo_StateSignal_mds.dph_abs_fil;
    }
    mcRpo_StateSignal_mds.sp_fir_ind = 0;
    mcRpo_StateSignal_mds.flx_arg_mem = mcRpo_StateSignal_mds.flx_arg;
}


/* !\brief Set base parameters
 * 
 * Details.
 * Set base parameters 
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
void mcRpo_BaseParametersSet( float samfreq, float basespe,  float basevol, float basecur )
{
    float f32a;

    /* base values setting */
    mcRpo_Parameters_mds.f32_sam_fre = samfreq;
    mcRpo_Parameters_mds.f32_bas_spe = basespe;

    /* internal useful conversion constants calculation */
    mcRpo_Parameters_mds.f32_k_volam = 1.0f / (basevol * basecur);
 #ifdef  CROSS_COUPLING_ENABLED
    mcRpo_Parameters_mds.f32_k_speed = (BASE_VALUE_FL) / basespe;
 #endif

    /* minimum speed in internal units */
    f32a = (OBS_MINSPEED_R_S * (BASE_VALUE_FL)) / basespe;
    mcRpo_StateSignal_mds.speed_min = (int16_t)f32a;
    /* minimum phase difference */
    f32a = OBS_MINFREQ_HZ * 65536.0 / mcRpo_Parameters_mds.f32_sam_fre;
    mcRpo_StateSignal_mds.dph_min = (uint16_t)f32a;

    /* conversion constants between speed in internal units and speed as
       filtered phase difference */
    f32a = ((32768.0f / FLOAT_PI)) * (mcRpo_Parameters_mds.f32_bas_spe / mcRpo_Parameters_mds.f32_sam_fre);
    mcRpo_StateSignal_mds.k_spe12 = (int16_t)f32a;
}


/* !\brief Set observer coefficients 
 * 
 * Details.
 * Set Observer coefficients
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
void mcRpo_SetObserverCoefficients(float rsta, float lsyn)
{
    float  f32a;

    /* preliminary floating point calculations */
    mcRpo_Parameters_mds.f32_sta_res = rsta;
    mcRpo_Parameters_mds.f32_syn_ind = lsyn;
    mcRpo_Parameters_mds.f32_k_gain = OBS_H_GAIN * mcRpo_Parameters_mds.f32_syn_ind * mcRpo_Parameters_mds.f32_sam_fre; /* always positive */
    mcRpo_Parameters_mds.f32_c1_coe = mcRpo_Parameters_mds.f32_k_gain - rsta;     /* can be negative */

    /* l11 calculation  */
    f32a = OBS_C0_GAIN;
    mcLib_FloatToIQ( f32a, &mcRpo_ObserverCoefficients_mds.l11 );

    /* m11 calculation  */
    f32a = mcRpo_Parameters_mds.f32_c1_coe * mcRpo_Parameters_mds.f32_k_volam * OBS_H_GAIN;
    mcLib_FloatToIQ( f32a, &mcRpo_ObserverCoefficients_mds.m11 );

    /* n11 calculation  */
    f32a = OBS_H_GAIN;
   mcLib_FloatToIQ( f32a,  &mcRpo_ObserverCoefficients_mds.n11 );

    /* k11 calculation  */
    f32a = mcRpo_Parameters_mds.f32_k_gain * mcRpo_Parameters_mds.f32_k_volam;
    mcLib_FloatToIQ( f32a, &mcRpo_ObserverCoefficients_mds.k11 );

 #ifdef  CROSS_COUPLING_ENABLED

    /* hhh calculation */
    f32a = mcRpo_Parameters_mds.f32_c1_coe * mcRpo_Parameters_mds.f32_k_volam;
    mcLib_FloatToIQ( f32a, &mcRpo_ObserverCoefficients_mds.hhh );

    /* .lll calculation  */
    f32a = mcRpo_Parameters_mds.f32_syn_ind * mcRpo_Parameters_mds.f32_k_volam / mcRpo_Parameters_mds.f32_k_speed;
    mcLib_FloatToIQ( f32a, &mcRpo_ObserverCoefficients_mds.lll );

    /* xxx calculation  */
    f32a = (1.0f / (mcRpo_Parameters_mds.f32_sam_fre * mcRpo_Parameters_mds.f32_k_speed));
    mcLib_FloatToIQ( f32a, &mcRpo_ObserverCoefficients_mds.xxx );

 #endif  // ifdef CROSS_COUPLING_ENABLED

 /* speed estimation init */
 #ifdef PH_CLAMP
    mcRpo_StateSignal_mds.max_dspeed = mcRpo_StateSignal_mds.max_dspeed;
 #endif 
 
}

/* !\brief Estimate observer coefficients 
 * 
 * Details.
 * Estimate Observer coefficients 
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
#ifdef RAM_EXECUTE
void __ramfunc__ mcRpo_EstimateObserverCoefficients(int16_t spref)
#else
void mcRpo_EstimateObserverCoefficients(int16_t spref)
#endif
{
 #ifdef  CROSS_COUPLING_ENABLED
    int32_t s32a;
    int16_t spabs;
 #endif /* ifdef CROSS_COUPLING_ENABLED */

    /* sign management */
    if(0 > spref)
    {
        mcRpo_StateSignal_mds.speed_sgn = -1;
    }
    else
    {
        mcRpo_StateSignal_mds.speed_sgn = 1;
    }

 #ifdef  CROSS_COUPLING_ENABLED

    /* speed clamp */
    /* 10.1 violation */
    if(mcRpo_StateSignal_mds.speed_min > mcRpo_StateSignal_mds.speed_abs)
    {
        spabs = mcRpo_StateSignal_mds.speed_min;
    }
    else
    {
        spabs = mcRpo_StateSignal_mds.speed_abs;
    }

    /* variable coefficient (mcRpo_ObserverCoefficients_mds.n21) (mcRpo_ObserverCoefficients_mds.xxx.val is positive) */
    s32a = ((int32_t)(mcRpo_ObserverCoefficients_mds.xxx.val)) * ((int32_t)spabs);
    mcRpo_ObserverCoefficients_mds.n21.shr = mcRpo_ObserverCoefficients_mds.xxx.shr;
    while((32767 < s32a) || (OBS_MAXSHIFTS < mcRpo_ObserverCoefficients_mds.n21.shr))
    {
        s32a >>= 1;
        mcRpo_ObserverCoefficients_mds.n21.shr--;   // if negative algo. will fail
    }
    mcRpo_ObserverCoefficients_mds.n21.val = (int16_t)s32a;
    while((0 == (mcRpo_ObserverCoefficients_mds.n21.val & 0x0001)) && (0 < mcRpo_ObserverCoefficients_mds.n21.shr))
    {
        mcRpo_ObserverCoefficients_mds.n21.val >>= 1;
        mcRpo_ObserverCoefficients_mds.n21.shr--;
    }

    /* variable coefficient (mcRpo_ObserverCoefficients_mds.m21) (mcRpo_ObserverCoefficients_mds.hhh.val can be negative) */
    s32a = ((int32_t)(mcRpo_ObserverCoefficients_mds.hhh.val)) * ((int32_t)(mcRpo_ObserverCoefficients_mds.n21.val));
    mcRpo_ObserverCoefficients_mds.m21.shr = mcRpo_ObserverCoefficients_mds.n21.shr + mcRpo_ObserverCoefficients_mds.hhh.shr;
    while(((32767 < s32a) || (-32767 > s32a)) || (OBS_MAXSHIFTS < mcRpo_ObserverCoefficients_mds.m21.shr))
    {
        s32a >>= 1;
        mcRpo_ObserverCoefficients_mds.m21.shr--;   // if negative algo. will fail
    }
    mcRpo_ObserverCoefficients_mds.m21.val = (int16_t)s32a;
    while((0 == (mcRpo_ObserverCoefficients_mds.m21.val & 0x0001)) && (0 < mcRpo_ObserverCoefficients_mds.m21.shr))
    {
        mcRpo_ObserverCoefficients_mds.m21.val >>= 1;
        mcRpo_ObserverCoefficients_mds.m21.shr--;
    }

    /* variable coefficient (mcRpo_ObserverCoefficients_mds.k21) (mcRpo_ObserverCoefficients_mds.lll.val is positive) */
    s32a = ((int32_t)(mcRpo_ObserverCoefficients_mds.lll.val)) * ((int32_t)spabs);
    mcRpo_ObserverCoefficients_mds.k21.shr = mcRpo_ObserverCoefficients_mds.lll.shr;
    while((32767 < s32a) || (OBS_MAXSHIFTS < mcRpo_ObserverCoefficients_mds.k21.shr))
    {
        s32a >>= 1;
        mcRpo_ObserverCoefficients_mds.k21.shr--;   /* if negative algo. will fail */
    }
    mcRpo_ObserverCoefficients_mds.k21.val = (int16_t)s32a;
    while((0 == (mcRpo_ObserverCoefficients_mds.k21.val & 0x0001)) && (0 < mcRpo_ObserverCoefficients_mds.k21.shr))
    {
        mcRpo_ObserverCoefficients_mds.k21.val >>= 1;
        mcRpo_ObserverCoefficients_mds.k21.shr--;
    }

    /* coefficients sign management */
    if(0 > mcRpo_StateSignal_mds.speed_sgn)
    {
        mcRpo_ObserverCoefficients_mds.n21.val = -mcRpo_ObserverCoefficients_mds.n21.val;
        mcRpo_ObserverCoefficients_mds.m21.val = -mcRpo_ObserverCoefficients_mds.m21.val;
        mcRpo_ObserverCoefficients_mds.k21.val = -mcRpo_ObserverCoefficients_mds.k21.val;
    }

 #endif  /* ifdef CROSS_COUPLING_ENABLED */
}

/* !\brief Arithmetic shift 
 * 
 * Details.
 * Arithmetic shift down, which goes to zero also with negative
 * numbers, but keeps output >=1 (<=-1) if the input is >1 (<-1)
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
#ifdef RAM_EXECUTE
int32_t __ramfunc__ shfdw2(int32_t a, uint16_t s)
#else
int32_t shfdw2(int32_t a, uint16_t s)
#endif
{
    int32_t r;

    if((-2 < a) && (2 > a))
    {
         r = 0;
    }
    else
    {
        r = a >> s;
        if(0 == r)
        {
            r = 1;
        }
    }
    return (r);
}


/* !\brief Multiply and  shift 
 * 
 * Details.
 * Multiply and shift
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
static inline int32_t mulshr(int16_t a, const IQ *c)
{
    int32_t r;

    r = ((int32_t)a) * ((int32_t)(c->val));
    r >>= (c->shr);

    return (r);
}



/* !\brief Execute Luenberger Observer
 * 
 * Details.
 * Execute Luenberger Observer
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */

#ifdef RAM_EXECUTE
void __ramfunc__ mcRpo_LuenbergerObserver( void )
#else
void mcRpo_LuenbergerObserver( void )
#endif
{
    int32_t s32x, s32y;
 #ifdef AMP_CLAMP
    int32_t luzx, luzy;
 #endif 

    tmcLib_Cartesian_s * Z;
    tmcLib_Cartesian_s * E;

    Z = &mcRpo_StateSignal_mds.obs_z;
    E = &mcRpo_StateSignal_mds.obs_e;

    mcRpo_EstimateObserverCoefficients(mcMocI_DirectionSign_gds16);

    s32x =  mulshr( Z->x, &mcRpo_ObserverCoefficients_mds.l11);
    s32x +=  mulshr(( mcRpo_InputSignal_mds.ia ), &mcRpo_ObserverCoefficients_mds.m11);
 #ifdef  CROSS_COUPLING_ENABLED
    s32x -= mulshr(( mcRpo_InputSignal_mds.ib ), &mcRpo_ObserverCoefficients_mds.m21); 
 #endif  // ifdef CROSS_COUPLING_ENABLED
    s32x +=  mulshr((mcRpo_StateSignal_mds.ua), &mcRpo_ObserverCoefficients_mds.n11);
 #ifdef  CROSS_COUPLING_ENABLED
    s32x -= mulshr((mcRpo_StateSignal_mds.ub), &mcRpo_ObserverCoefficients_mds.n21); 
 #endif  // ifdef CROSS_COUPLING_ENABLED

    s32y = mulshr(Z->y, &mcRpo_ObserverCoefficients_mds.l11); 
 #ifdef  CROSS_COUPLING_ENABLED
    s32y += mulshr(( mcRpo_InputSignal_mds.ia ), &mcRpo_ObserverCoefficients_mds.m21);
 #endif  // ifdef CROSS_COUPLING_ENABLED
    s32y += mulshr(( mcRpo_InputSignal_mds.ib ), &mcRpo_ObserverCoefficients_mds.m11);
 #ifdef  CROSS_COUPLING_ENABLED
    s32y += mulshr((mcRpo_StateSignal_mds.ua), &mcRpo_ObserverCoefficients_mds.n21);
 #endif  // ifdef CROSS_COUPLING_ENABLED
    s32y += mulshr((mcRpo_StateSignal_mds.ub), &mcRpo_ObserverCoefficients_mds.n11); 

 #ifdef AMP_CLAMP
    /* during startup transitory from zero speed, observer status could diverge;
       this clamping keeps the vector amplitude clamped, without changing its argument */
    luzx = s32x;
    luzy = s32y;
    while((-32767 > luzx) || (32767 < luzx) || (-32767 > luzy) || (32767 < luzy))
    {
     luzx = mcLib_RightShift(luzx, 1);
     luzy = mcLib_RightShift(luzy, 1);
     debug_cnt1++;
    }
    Z->x = (int16_t)luzx;
    Z->.y = (int16_t)luzy;
 #else /* ifdef AMP_CLAMP */
    Z->x = (int16_t)s32x;
    Z->y = (int16_t)s32y;
 #endif /* ifdef AMP_CLAMP */

    s32x -= mulshr(( mcRpo_InputSignal_mds.ia ), &mcRpo_ObserverCoefficients_mds.k11);
 #ifdef  CROSS_COUPLING_ENABLED
    s32x += mulshr(( mcRpo_InputSignal_mds.ib ), &mcRpo_ObserverCoefficients_mds.k21); 
 #endif  // ifdef CROSS_COUPLING_ENABLED

 #ifdef  CROSS_COUPLING_ENABLED
    s32y -= mulshr(( mcRpo_InputSignal_mds.ia ), &mcRpo_ObserverCoefficients_mds.k21);
 #endif  // ifdef CROSS_COUPLING_ENABLED
    s32y -=  mulshr(( mcRpo_InputSignal_mds.ib ), &mcRpo_ObserverCoefficients_mds.k11); 

 #ifdef AMP_CLAMP
    while((-32767 > s32x) || (32767 < s32x) || (-32767 > s32y) || (32767 < s32y))
    {
     s32x = mcLib_RightShift(s32x, 1);
     s32y = mcLib_RightShift(s32y, 1);
     debug_cnt2++;
    }
 #endif // ifdef AMP_CLAMP

    E->x = (int16_t)s32x;
    E->y = (int16_t)s32y;

    mcRpo_StateSignal_mds.ua = mcRpo_InputSignal_mds.ua;
    mcRpo_StateSignal_mds.ub = mcRpo_InputSignal_mds.ub;
  
}


/* !\brief Phase Estimation
 * 
 * Details.
 * Phase Estimation
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
#ifdef RAM_EXECUTE
void __ramfunc__ mcRpo_PhaseEstimation(void)
#else
void mcRpo_PhaseEstimation(void)
#endif
{
    uint16_t u16a;
 #ifdef PH_CLAMP
    uint16_t u16b;
    uint16_t u16c;
 #endif /* ifdef PH_CLAMP */

 #ifdef PH_CLAMP
    /* phase variation limits calculation */
    u16a = mcRpo_StateSignal_mds.dph_abs_fil >> 1;
    if(mcRpo_StateSignal_mds.max_dspeed < u16a)
    {
        u16b = mcRpo_StateSignal_mds.dph_abs_fil - mcRpo_StateSignal_mds.max_dspeed;
        u16c = mcRpo_StateSignal_mds.dph_abs_fil + mcRpo_StateSignal_mds.max_dspeed;
    }
    else
    {
        u16b = mcRpo_StateSignal_mds.dph_abs_fil - u16a;
        u16c = mcRpo_StateSignal_mds.dph_abs_fil + u16a;
    }
 #endif /* ifdef PH_CLAMP */

    mcLib_Cartesian2Polar(&mcRpo_StateSignal_mds.obs_e, &mcRpo_StateSignal_mds.bemf);  /* extract angular position */
    if(0 > mcRpo_StateSignal_mds.speed_sgn)
    {
        u16a = mcRpo_StateSignal_mds.bemf.t.ang + PIHALVES; /* overflow is OK here */
     #ifndef PH_CLAMP
        if((mcRpo_StateSignal_mds.flx_arg - u16a) > (uint16_t)PI)
        {
            u16a = mcRpo_StateSignal_mds.flx_arg;             /* cannot increase */
        }
     #else   /* ifndef PH_CLAMP */
        u16b = mcRpo_StateSignal_mds.flx_arg - u16b;   /* overflow is OK here */
        u16c = mcRpo_StateSignal_mds.flx_arg - u16c;   /* overflow is OK here */
        if(((uint16_t)(u16b - u16a)) > PI)
        {
            u16a = u16b;
        }
        else if((u16a - u16c) > (uint16_t)PI)
        {
            u16a = u16c;
        }
     #endif
    }
    else
    {
        u16a = mcRpo_StateSignal_mds.bemf.t.ang - PIHALVES; /* overflow is OK here */
     #ifndef PH_CLAMP
        if((u16a - mcRpo_StateSignal_mds.flx_arg) > (uint16_t)PI)
        {
           u16a = mcRpo_StateSignal_mds.flx_arg;             /* cannot decrease */
        }
     #else   /* ifndef PH_CLAMP */
        u16b = mcRpo_StateSignal_mds.flx_arg + u16b;   /* overflow is OK here */
        u16c = mcRpo_StateSignal_mds.flx_arg + u16c;   /* overflow is OK here */
        if((u16a - u16b) > (uint16_t)PI)
        {
            u16a = u16b;
        }
        else if((u16c - u16a) > (uint16_t)PI)
        {
            u16a = u16c;
        }
     #endif /* ifdef PH_CLAMP */
    }
    mcRpo_StateSignal_mds.flx_arg = u16a;
}


/* !\brief Speed Estimation
 * 
 * Details.
 * Speed Estimation
 * 
 * @param[in]: None 
 * @param[in/out]: None
 * @param[out]: None 
 * @return: Angular position 
 */
#ifdef RAM_EXECUTE
void __ramfunc__ mcRpo_SpeedEstimation(void)
#else
void mcRpo_SpeedEstimation(void)
#endif
{
 int16_t dph;

 /* sign management and delta ang clamp */
 if(0 > mcRpo_StateSignal_mds.speed_sgn)
 {
  dph = (int16_t)mcRpo_StateSignal_mds.flx_arg_mem - (int16_t)mcRpo_StateSignal_mds.flx_arg;
 }
 else
 {
 dph = (int16_t)mcRpo_StateSignal_mds.flx_arg - (int16_t)mcRpo_StateSignal_mds.flx_arg_mem;
 }
 mcRpo_StateSignal_mds.flx_arg_mem = mcRpo_StateSignal_mds.flx_arg;
 if(1 > dph)
 {
  dph = mcRpo_StateSignal_mds.dph_min;
 }
  mcRpo_StateSignal_mds.dph_global = dph;
 /* first filter (FIR) */
 /* since we use as output the accumulator undivided, the amplification is
  4=2^2 if we calculate the speed over 4 samples; if the speed is
  calculated over a different number of samples, the amplification has
  to be adapted in consequence, since at the end we want a total amp of
  2^14 */
 mcRpo_StateSignal_mds.sp_fir_acc += dph;
 mcRpo_StateSignal_mds.sp_fir_acc -= mcRpo_StateSignal_mds.sp_fir_vec[mcRpo_StateSignal_mds.sp_fir_ind];
 mcRpo_StateSignal_mds.sp_fir_vec[mcRpo_StateSignal_mds.sp_fir_ind] = dph;  /* max speed: pi[rad/s]/Ts[s] */
    mcRpo_StateSignal_mds.sp_fir_ind++;
 /* mcRpo_StateSignal_mds.sp_fir_ind &= 0x07; speed calculated over 8 samples */
 mcRpo_StateSignal_mds.sp_fir_ind &= 0x03; /* speed calculated over 4 samples */

 /* now we will apply three IIR in cascade configuration;
  the IIR time constant is ((2^4)-1)*Ts, so the cut-off frequency is Fs/(30pi)
  (around 85Hz if Fs=8kHz) */

 /* second filter (IIR) */
 /* since we use as output the filter memory, the amplification is 2^4=16 */
 /*mcRpo_StateSignal_mds.sp_iir1_mem -= shfdw2(mcRpo_StateSignal_mds.sp_iir1_mem, 4);*/ /* mcRpo_StateSignal_mds.sp_iir1_mem -= mcRpo_StateSignal_mds.sp_iir1_mem >> 4; */

    mcRpo_StateSignal_mds.sp_iir1_mem -= mcRpo_StateSignal_mds.sp_iir1_mem >> 4;
 /* mcRpo_StateSignal_mds.sp_iir1_mem += shfdw1(mcRpo_StateSignal_mds.sp_fir_acc, 1); speed calculated over 8 samples */
 mcRpo_StateSignal_mds.sp_iir1_mem += mcRpo_StateSignal_mds.sp_fir_acc; /* speed calculated over 4 samples */

 /* third filter (IIR) */
 /* since we use as output the filter memory, the amplification is 2^4=16 */
 /*mcRpo_StateSignal_mds.sp_iir2_mem -= shfdw2(mcRpo_StateSignal_mds.sp_iir2_mem, 4);*/ /* mcRpo_StateSignal_mds.sp_iir2_mem -= mcRpo_StateSignal_mds.sp_iir2_mem >> 4; */
 mcRpo_StateSignal_mds.sp_iir2_mem -= mcRpo_StateSignal_mds.sp_iir2_mem >> 4;
 mcRpo_StateSignal_mds.sp_iir2_mem += mcRpo_StateSignal_mds.sp_iir1_mem;

 /* fourth filter (IIR) */
 /* since we use as output the filter memory, the amplification is 2^4=16 */
 /*mcRpo_StateSignal_mds.sp_iir3_mem -= shfdw2(mcRpo_StateSignal_mds.sp_iir3_mem, 4);*/ /* mcRpo_StateSignal_mds.sp_iir3_mem -= mcRpo_StateSignal_mds.sp_iir3_mem >> 4; */
 mcRpo_StateSignal_mds.sp_iir3_mem -= mcRpo_StateSignal_mds.sp_iir3_mem >> 4;
 mcRpo_StateSignal_mds.sp_iir3_mem += mcRpo_StateSignal_mds.sp_iir2_mem;

 /* the total amplification is 2^(2+3*4=14); now come back to the internal units */
 mcRpo_StateSignal_mds.speed_abs = (int16_t)(mcRpo_StateSignal_mds.sp_iir3_mem / mcRpo_StateSignal_mds.k_spe12);
 #ifdef PH_CLAMP
 mcRpo_StateSignal_mds.dph_abs_fil = (int16_t)(mcRpo_StateSignal_mds.sp_iir3_mem >> 14);
 #endif 
 if(0 > mcRpo_StateSignal_mds.speed_sgn)
 {
  mcRpo_StateSignal_mds.speed_est = -mcRpo_StateSignal_mds.speed_abs;
 }
 else
 {
  mcRpo_StateSignal_mds.speed_est = mcRpo_StateSignal_mds.speed_abs;
 }
}


/* !\brief Read input signals for  Observer 
 * 
 * Details.
 * This local function reads inputs from external modules and updates the static input 
 * structure.
 * 
 * @param[in]: 
 * @param[in/out]: 
 * @param[out]: 
 * @return: None
 */
#ifdef RAM_EXECUTE
void __ramfunc__ mcRpo_ReadInputSignal( void )
#else
__STATIC_INLINE void mcRpo_ReadInputSignal( void )
#endif
{
    /* Read input signals  */  
    mcRpo_InputSignal_mds.ia = mcCur_MeasuredAlphaAxisCurrent_gds16;
    mcRpo_InputSignal_mds.ib = mcCur_MeasuredBetaAxisCurrent_gds16;
    mcRpo_InputSignal_mds.ua = mcPwmI_AlphaAxisVoltage_gds;
    mcRpo_InputSignal_mds.ub = mcPwmI_BetaAxisVoltage_gds;
 }

/* !\brief Speed dependent angle compensation
 * 
 * Details.
 * This function performs speed dependent compensation for the time elapsed between
 *  input signals reading and the PWM application
 * 
 * @param[in]: 
 * @param[in/out]: 
 * @param[out]: 
 * @return: None
 */
#ifdef RAM_EXECUTE
uint16_t __ramfunc__ mcRpo_DelayCompensation(void)
#else
uint16_t mcRpo_DelayCompensation(void)
#endif
{
 int32_t s32a;
 int16_t s16a;
 uint16_t retval;

 s32a = mcRpo_StateSignal_mds.sp_iir3_mem; 
 if((int32_t)BASE_VALUE <= s32a)
 {
  if(0 > mcRpo_StateSignal_mds.speed_sgn)
  {
     s16a = (int16_t)(-(s32a >> SH_BASE_VALUE));
     retval = ((uint16_t)s16a);
  }
  else
  {
         s16a = (int16_t)((s32a >> SH_BASE_VALUE));
         retval = ((uint16_t)s16a);
  }
 }
 else
 {
          retval = (0U);
 }
 return (retval);
}


/* !\brief Write output signals of the  Observer 
 * 
 * Details.
 * This local function writes calculated outputs from output structure to global 
 *  interface variables.
 * 
 * @param[in]: 
 * @param[in/out]: 
 * @param[out]: 
 * @return: None
 */
#ifdef RAM_EXECUTE
void __ramfunc__ mcRpo_WriteOutputSignal( void )
#else
__STATIC_INLINE void mcRpo_WriteOutputSignal( void )
#endif
{
    mcRpo_OutputSignal_mds.angle =  mcRpo_GetAngularPosition();
    mcRpo_OutputSignal_mds.speed =   mcRpo_GetAngularSpeed();
    mcRpoI_ElectricalRotorPosition_gdu16 =  mcRpo_OutputSignal_mds.angle;
    mcRpoI_ElectricalRotorSpeed_gds16 =  mcRpo_OutputSignal_mds.speed;
    mcRpoI_BacKEmfMagnitude_gds16  = mcRpo_OutputSignal_mds.Es;
}

/*******************************************************************************
 * Interface Functions
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
void mcRpoI_InitializeObserver( void )
 {
      mcRpo_BaseParametersSet(CURRENT_CONTROL_FREQUENCY, BASE_SPEED,   BASE_VOLTAGE, BASE_CURRENT);
      mcRpo_SetObserverCoefficients( MOTOR_PER_PHASE_RESISTANCE_IN_OHM, QUADRATURE_AXIS_INDUCTANCE_IN_HENRY);
      mcRpo_ResetObserver();
 }

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
void __ramfunc__ MCRPO_ExecuteEstimator( void )
#else
void MCRPO_ExecuteEstimator( void )
#endif
{
    /* Read input signal */
    mcRpo_ReadInputSignal();
    
    /* Execute Luenberger Observer */
     mcRpo_LuenbergerObserver();
    
    /* Phase Estimation */
    mcRpo_PhaseEstimation();
    
    /* Speed Estimation */
    mcRpo_SpeedEstimation();

    /* Write output signal */
    mcRpo_WriteOutputSignal();

}

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
uint16_t __ramfunc__ mcRpo_GetAngularPosition(void)
#else
uint16_t mcRpo_GetAngularPosition(void)
#endif
{
    return(mcRpo_StateSignal_mds.flx_arg + mcRpo_DelayCompensation());
}

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
int16_t __ramfunc__ mcRpo_GetAngularSpeed(void)
#else
int16_t mcRpo_GetAngularSpeed(void)
#endif
{
    return(mcRpo_StateSignal_mds.speed_est);
}


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
void mcRpo_ResetObserver( void )
{
    mcRpo_SpeedFilterInit(mcMocI_DirectionSign_gds16 );

    /* other internal variables init */
    mcRpo_StateSignal_mds.flx_arg_mem = 0;
    mcRpo_StateSignal_mds.flx_arg = 0;
    mcRpo_StateSignal_mds.obs_e.x = 0;
    mcRpo_StateSignal_mds.obs_e.y = 0;
    mcRpo_StateSignal_mds.bemf.r = 0;
    mcRpo_StateSignal_mds.bemf.t.ang = 0;
    mcRpo_StateSignal_mds.bemf.t.sin = 0;
    mcRpo_StateSignal_mds.bemf.t.cos = (int16_t)BASE_VALUE;

     /* observer status init */
    mcRpo_StateSignal_mds.obs_z.x = 0;
    mcRpo_StateSignal_mds.obs_z.y = 0;
}