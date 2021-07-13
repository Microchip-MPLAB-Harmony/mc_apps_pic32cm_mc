/*******************************************************************************
 Generic Motor Control Library 

  Company:
    Microchip Technology Inc.

  File Name:
    mc_math_library.c

  Summary:
    Math Library implemented in Q14 fixed point arithmetic.

  Description:
    This file implements generic vector motor control related functions
     like Trigonometric
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

#include <math.h>
#include <stdint.h>
#include "mc_math_library.h"
#include "definitions.h"
#include "mc_userparams.h"

/*******************************************************************************
Macro definitions
*******************************************************************************/


#define OBS_MAXSHIFTS		( 32 )

/* trigonometric tables */
#define SH_TRITAB_DIM	( 8U )
#define TRITAB_DIM		( (uint16_t)1U << (uint16_t)SH_TRITAB_DIM )
#define SH_SINTAB		( 14U - SH_TRITAB_DIM )	// PIHALVES=(2^16)/4=2^14
#define SH_SACTAB		( SH_BASE_VALUE - SH_TRITAB_DIM )
#define SH_ACTTAB		( SH_BASE_VALUE - SH_TRITAB_DIM )
#define	SEL1Q			( 0x3FFFU )	/* select first_quarter_value */
#define	ISCOS			( 0x4000U )	/* table(first_quarter_value) gives cos */
#define	ISNEG			( 0x8000U )	/* sin is neg */


/******************************************************************************
Private global variables
******************************************************************************/

/* table y = BASE_VALUE * sin((pi/2) * x / TRITAB_DIM)
 0 <= x <= TRITAB_DIM, 0 <= y <= BASE_VALUE (first quarter) */
static const int16_t sin_table[TRITAB_DIM + 1U] = {
	    0,   101,   201,   302,   402,   503,   603,   704,	//   0, ..,   7
	  804,   904,  1005,  1105,  1205,  1306,  1406,  1506,	//   8, ..,  15
	 1606,  1706,  1806,  1906,  2006,  2105,  2205,  2305,	//  16, ..,  23
	 2404,  2503,  2603,  2702,  2801,  2900,  2999,  3098,	//  24, ..,  31
	 3196,  3295,  3393,  3492,  3590,  3688,  3786,  3883,	//  32, ..,  39
	 3981,  4078,  4176,  4273,  4370,  4467,  4563,  4660,	//  40, ..,  47
	 4756,  4852,  4948,  5044,  5139,  5235,  5330,  5425,	//  48, ..,  55
	 5520,  5614,  5708,  5803,  5897,  5990,  6084,  6177,	//  56, ..,  63
	 6270,  6363,  6455,  6547,  6639,  6731,  6823,  6914,	//  64, ..,  71
	 7005,  7096,  7186,  7276,  7366,  7456,  7545,  7635,	//  72, ..,  79
	 7723,  7812,  7900,  7988,  8076,  8163,  8250,  8337,	//  80, ..,  87
	 8423,  8509,  8595,  8680,  8765,  8850,  8935,  9019,	//  88, ..,  95
	 9102,  9186,  9269,  9352,  9434,  9516,  9598,  9679,	//  96, .., 103
	 9760,  9841,  9921, 10001, 10080, 10159, 10238, 10316,	// 104, .., 111
	10394, 10471, 10549, 10625, 10702, 10778, 10853, 10928,	// 112, .., 119
	11003, 11077, 11151, 11224, 11297, 11370, 11442, 11514,	// 120, .., 127
	11585, 11656, 11727, 11797, 11866, 11935, 12004, 12072,	// 128, .., 135
	12140, 12207, 12274, 12340, 12406, 12472, 12537, 12601,	// 136, .., 143
	12665, 12729, 12792, 12854, 12916, 12978, 13039, 13100,	// 144, .., 151
	13160, 13219, 13279, 13337, 13395, 13453, 13510, 13567,	// 152, .., 159
	13623, 13678, 13733, 13788, 13842, 13896, 13949, 14001,	// 160, .., 167
	14053, 14104, 14155, 14206, 14256, 14305, 14354, 14402,	// 168, .., 175
	14449, 14497, 14543, 14589, 14635, 14680, 14724, 14768,	// 176, .., 183
	14811, 14854, 14896, 14937, 14978, 15019, 15059, 15098,	// 184, .., 191
	15137, 15175, 15213, 15250, 15286, 15322, 15357, 15392,	// 192, .., 199
	15426, 15460, 15493, 15525, 15557, 15588, 15619, 15649,	// 200, .., 207
	15679, 15707, 15736, 15763, 15791, 15817, 15843, 15868,	// 208, .., 215
	15893, 15917, 15941, 15964, 15986, 16008, 16029, 16049,	// 216, .., 223
	16069, 16088, 16107, 16125, 16143, 16160, 16176, 16192,	// 224, .., 231
	16207, 16221, 16235, 16248, 16261, 16273, 16284, 16295,	// 232, .., 239
	16305, 16315, 16324, 16332, 16340, 16347, 16353, 16359,	// 240, .., 247
	16364, 16369, 16373, 16376, 16379, 16381, 16383, 16384,	// 248, .., 255
	16384};	
	
	
/* table y = ((PI / FLOAT_PI) * atan(x / TRITAB_DIM))
 0 <= x <= TRITAB_DIM, 0 <= y <= PIFOURTHS (first half quarter) */
static const int16_t library_tbact[TRITAB_DIM + 1U] = {
	    0,    41,    81,   122,   163,   204,   244,   285, //   0, ..,   7
	  326,   367,   407,   448,   489,   529,   570,   610, //   8, ..,  15
	  651,   692,   732,   773,   813,   854,   894,   935, //  16, ..,  23
	  975,  1015,  1056,  1096,  1136,  1177,  1217,  1257, //  24, ..,  31
	 1297,  1337,  1377,  1417,  1457,  1497,  1537,  1577, //  32, ..,  39
	 1617,  1656,  1696,  1736,  1775,  1815,  1854,  1894, //  40, ..,  47
	 1933,  1973,  2012,  2051,  2090,  2129,  2168,  2207, //  48, ..,  55
	 2246,  2285,  2324,  2363,  2401,  2440,  2478,  2517, //  56, ..,  63
	 2555,  2594,  2632,  2670,  2708,  2746,  2784,  2822, //  64, ..,  71
	 2860,  2897,  2935,  2973,  3010,  3047,  3085,  3122, //  72, ..,  79
	 3159,  3196,  3233,  3270,  3307,  3344,  3380,  3417, //  80, ..,  87
	 3453,  3490,  3526,  3562,  3599,  3635,  3670,  3706, //  88, ..,  95
	 3742,  3778,  3813,  3849,  3884,  3920,  3955,  3990, //  96, .., 103
	 4025,  4060,  4095,  4129,  4164,  4199,  4233,  4267, // 104, .., 111
	 4302,  4336,  4370,  4404,  4438,  4471,  4505,  4539, // 112, .., 119
	 4572,  4605,  4639,  4672,  4705,  4738,  4771,  4803, // 120, .., 127
	 4836,  4869,  4901,  4933,  4966,  4998,  5030,  5062, // 128, .., 135
	 5094,  5125,  5157,  5188,  5220,  5251,  5282,  5313, // 136, .., 143
	 5344,  5375,  5406,  5437,  5467,  5498,  5528,  5559, // 144, .., 151
	 5589,  5619,  5649,  5679,  5708,  5738,  5768,  5797, // 152, .., 159
	 5826,  5856,  5885,  5914,  5943,  5972,  6000,  6029, // 160, .., 167
	 6058,  6086,  6114,  6142,  6171,  6199,  6227,  6254, // 168, .., 175
	 6282,  6310,  6337,  6365,  6392,  6419,  6446,  6473, // 176, .., 183
	 6500,  6527,  6554,  6580,  6607,  6633,  6660,  6686, // 184, .., 191
	 6712,  6738,  6764,  6790,  6815,  6841,  6867,  6892, // 192, .., 199
	 6917,  6943,  6968,  6993,  7018,  7043,  7068,  7092, // 200, .., 207
	 7117,  7141,  7166,  7190,  7214,  7238,  7262,  7286, // 208, .., 215
	 7310,  7334,  7358,  7381,  7405,  7428,  7451,  7475, // 216, .., 223
	 7498,  7521,  7544,  7566,  7589,  7612,  7635,  7657, // 224, .., 231
	 7679,  7702,  7724,  7746,  7768,  7790,  7812,  7834, // 232, .., 239
	 7856,  7877,  7899,  7920,  7942,  7963,  7984,  8005, // 240, .., 247
	 8026,  8047,  8068,  8089,  8110,  8131,  8151,  8172, // 248, .., 255
	 8192};							// 256



/******************************************************************************
 * Interface variables 
******************************************************************************/

/******************************************************************************
 * Interface Functions 
******************************************************************************/

/*! \brief Calculate sine value  
 * 
 * Details.
 * Calculate sine value 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */

#ifdef RAM_EXECUTE
int16_t __ramfunc__ mcLib_Sine(uint16_t ang)
#else
int16_t mcLib_Sine(uint16_t ang)
#endif
{
    uint16_t	a;
    int16_t	y;

    a = ang & SEL1Q; /* select angle in the first quarter (<= PIHALVES) */
    if((ISCOS & ang) != 0U)
    {
        a = PIHALVES - a;
    }
    y = sin_table[a >> SH_SINTAB];
    return (((ISNEG & ang) != 0U)? -y: y);
}

/*! \brief Calculate cosine value  
 * 
 * Details.
 * Calculate cosine value 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
#ifdef RAM_EXECUTE
int16_t __ramfunc__ mcLib_Cosine(uint16_t ang)
#else
int16_t mcLib_Cosine(uint16_t ang)
#endif
{
    uint16_t	a;
    int16_t		y;
    uint16_t  ang_temp;

    /* overflow is OK here due to angle periodicity */
    ang_temp = ang + PIHALVES;
    a = ang_temp  & SEL1Q;  /* select angle in the first quarter (<= PIHALVES) */
    if((ISCOS & ang_temp) != 0U)
    {
        a = PIHALVES - a;
    }
    y = sin_table[a >> SH_SINTAB];
    return (((ISNEG & ang_temp) != 0U )? -y: y);
}

/*! \brief Calculate inverse tangent value  
 * 
 * Details.
 * Calculate inverse tangent value 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
#ifdef RAM_EXECUTE
uint16_t  __ramfunc__ mcLib_InverseTangent(int16_t x, int16_t y)
#else
uint16_t mcLib_InverseTangent(int16_t x, int16_t y)
#endif
{
    uint32_t u32a;
    uint16_t u16a, off, sgn, a, b;

    if(0 == y)
    {
         if(0 > x)
         {
	    u16a = PI;
         }
         else
         {
	    u16a = 0;
         }
     }
     else if(0 == x)
     {
         if(0 > y)
         {
	    u16a = THREEPIHALVES;
         }
         else
         {
	    u16a = PIHALVES;
         }
     }
    else
    {
         if(0 > y)
	{
               if(-32768 == y)
               {
		b = 32767;
	     }
	    else
	    {
		b = (uint16_t)-y;
	    }
	   if(0 > x)	// x < 0, y < 0
	   {
	       if(-32768 == x)
	        {
		  a = 32767;
	        }
	        else
	        {
	  	 a = (uint16_t)-x;
	        }
	        off = PI;
	        sgn = 0;
	    }
	    else		// x > 0, y < 0
	    {
	        a = (uint16_t)x;
	        off = 0;
	        sgn = 1;
	    }
          }
	else
	{
	    b = (uint16_t)y;
	    if(0 > x)	// x < 0, y > 0
	    {
		if(-32768 == x)
		{
	              a = 32767;
	 	}
		else
		{
		    a = (uint16_t)-x;
		}
		off = PI;
		sgn = 1;
	     }
	     else		// x > 0, y > 0
	     {
	 	a = (uint16_t)x;
		off = 0;
		sgn = 0;
	      }
	}
	if(b == a)
	{
	     u16a = PIFOURTHS;
	}
	else if(b < a)
	{
	      u32a = (uint32_t)b * (uint32_t)BASE_VALUE;
                u16a = (uint16_t)(u32a / a);

                u16a = (uint16_t)library_tbact[u16a >> (uint16_t)SH_ACTTAB];
	}
	else
	{
	       u32a = (uint32_t)a * (uint32_t)BASE_VALUE;
                 u16a = (uint16_t)(u32a / b);

	       u16a = (uint16_t)library_tbact[u16a >> (uint16_t)SH_ACTTAB];
	       u16a = PIHALVES - u16a;	
                        /* overflow is OK here! */
	}
	if(0U != sgn)
	{
	     u16a = off - u16a;		// overflow is OK here!
	}
	else
	{
	    u16a = off + u16a;		// overflow is OK here!
	}
     }
     return(u16a);

}	/* end of function library_atan2(...) */

/*! \brief Calculate square root value   
 * 
 * Details.
 * Calculate square root value  
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
#ifdef RAM_EXECUTE
uint32_t  __ramfunc__ mcLib_SquareRoot(uint32_t number)
#else
uint32_t mcLib_SquareRoot(uint32_t number )
#endif 
{
    uint16_t u16a;
    u16a = (int16_t)DIVAS_SquareRoot( number );
    return u16a;
}

/*! \brief Calculate adjacent side  
 * 
 * Details.
 * Calculate adjacent side  
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
#ifdef RAM_EXECUTE
int16_t  __ramfunc__ mcLib_DetermineAdjSide(int16_t hypo, int16_t fcat)
#else
int16_t mcLib_DetermineAdjSide(int16_t hypo, int16_t fcat)
#endif 
{
    int16_t	s16a;
  
    if( hypo > fcat)
    {
       s16a = (int16_t)DIVAS_SquareRoot((uint32_t)(hypo * hypo) - (uint32_t)(fcat * fcat ));

    }
    else 
    {
        s16a = 0;
    }
  
   return(s16a);
}

/*! \brief Right shift  
 * 
 * Details.
 * Right shift 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
#ifdef RAM_EXECUTE
int32_t __ramfunc__ mcLib_RightShift(int32_t operand, uint16_t shift )
#else
int32_t mcLib_RightShift(int32_t operand, uint16_t shift )
#endif
{
    int32_t result;

    if(0 > operand )
    {
        result = -((-operand ) >> shift);
    }
    else
    {
        result = operand >> shift;
    }
    return (result);
}

/*! \brief Multiply and right shift  
 * 
 * Details.
 * Multiply and right shift 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
int32_t mcLib_MultShiftRight( const int16_t m1, const int16_t m2, const uint16_t sh )
{
    int32_t result;
    
    result = (int32_t)(((int32_t)((int32_t)m1 * (int32_t)m2)) >> sh );
	
    return result;
}


/*! \brief Float Scaling 
 * 
 * Details.
 * Float scaling 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcLib_FloatScaling( const float f32a, IQ * const iq, const uint8_t q)
{
    if( 0.0f < f32a )
    {
        iq->val = (int16_t)( f32a * (float)( 1 << q ));
        iq->shr = q;
    }
    else 
    {
        iq->val = -(int16_t)( (-f32a) * (float)( 1 << q ));
        iq->shr = q;
    }
}

/*! \brief IQ  multiplication
 * 
 * Details.
 * IQ multiplication 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcLib_IQMultiplication( IQ m1, IQ m2, IQ * result )
{
    int32_t  s32a;
    uint16_t u16a;

    s32a = (int32_t)((int32_t)m1.val * (int32_t)m2.val ) ;
    u16a = (uint16_t)( m1.shr + m2.shr );

    while( ( MAX_VALUE_S16 < s32a  ) || ( MIN_VALUE_S16 > s32a) || ( OBS_MAXSHIFTS < u16a ))
    {
        s32a >>= 1;
        u16a--;
    }
    while( ( 0 == ( s32a & 0x0001 )) && ( 0 < u16a ))
    {
        s32a >>= 1;
        u16a--;
    }

    result->val = (int16_t)s32a;
    result->shr = (uint16_t)u16a;

}



/*! \brief Cartesian to polar conversion  
 * 
 * Details.
 * Cartesian to polar conversion
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
#ifdef RAM_EXECUTE
void __ramfunc__ mcLib_Cartesian2Polar(const tmcLib_Cartesian_s *xy, tmcLib_Polar_s *rt)
#else
void mcLib_Cartesian2Polar(const tmcLib_Cartesian_s *xy, tmcLib_Polar_s *rt)
#endif
{
    int32_t	s32a;
    int16_t temp;

    ((rt->t).ang) = mcLib_InverseTangent(xy->x, xy->y);
    rt->t.sin =  mcLib_Sine(rt->t.ang);
    rt->t.cos =  mcLib_Cosine(rt->t.ang);
          
    if(	((PIFOURTHS < (rt->t).ang)	&& (THREEPIFOURTHS > (rt->t).ang)) ||
          ((FIVEPIFOURTHS < (rt->t).ang)	&& (SEVENPIFOURTHS > (rt->t).ang)) )
    {	
        /* |sin(ang)|>|cos(ang)| */
        s32a = ((int32_t)(xy->y)) * (int32_t)BASE_VALUE;
        temp =  (int16_t)(s32a / ((rt->t).sin)) ;
        (rt->r) = (uint16_t) temp;
    }
    else
    {
        /* |sin(ang)|<=|cos(ang)|*/
        s32a = ((int32_t)(xy->x)) * (int32_t)BASE_VALUE;
        temp =  (int16_t)(s32a / ((rt->t).cos)) ;
        (rt->r) = (uint16_t)temp;
     }
}

/*! \brief Float to IQ format conversion  
 * 
 * Details.
 * Float to IQ format conversion
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcLib_FloatToIQ( float f32a, IQ * const result )
{
     result->shr = 0u;   
    
    if(f32a >= 0.0f)
    {
        while((16383.0f > f32a) && ((uint16_t)OBS_MAXSHIFTS > result->shr))
        {
            f32a *= 2.0f;
            result->shr++;
        }
    }
    else
    {
        while((-16383.0f < f32a) && ((uint16_t)OBS_MAXSHIFTS > result->shr))
        {
            f32a *= 2.0f;
            result->shr++;
        }
    }
    result->val = (int16_t)f32a;        /* can be negative */
    while((0 == ( result->val & 0x0001)) && (0U < result->shr))
    {
        result->val >>= 1;
        result->shr--;
    }
}

