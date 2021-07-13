/*******************************************************************************
  System Definitions

  File Name:
    mc_math_library.h

  Summary:
    Header file which contains variables and function prototypes for math library.
 
  Description:
    This file contains variables and function prototypes for math library used in Motor Control.
    Implemented in Q2.14 Fixed Point Arithmetic.
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

#ifndef MC_MATH_H
#define MC_MATH_H

#include <stdint.h>
#include <sys/attribs.h>
#include "mc_userparams.h"


/******************************************************************************
 * Constants  
******************************************************************************/

#define   BASE_VALUE_FL             (float)( BASE_VALUE   )
#define   FLOAT_PI	                ( 3.141592654f )
#define   K_ANGLE		      (float)(TWOPI / (2.0 * FLOAT_PI))

/* Angle definitions [0, 65535 ] */
#define   PIFOURTHS		      (  8192U    )		
#define   PIHALVES		      ( 16384U   )		
#define   THREEPIFOURTHS	      ( 24576U  )		
#define   PI				      ( 32768U  )		
#define   FIVEPIFOURTHS	      ( 40960U  )		
#define   THREEPIHALVES	      ( 49152U  )		
#define   SEVENPIFOURTHS          ( 57344U  )		
#define    TWOPI			      ( 65536UL )		



#define MAX_VALUE_S16       32767
#define MIN_VALUE_S16      -32768

/******************************************************************************
 * User-defined data structure  
******************************************************************************/
typedef struct
{
    int16_t     val;
    uint16_t	shr;
}IQ;

typedef struct
{	
    int16_t	x;		// first component
    int16_t	y;		// second component
}tmcLib_Cartesian_s;

typedef struct
{
    uint16_t	 ang;	/* angle */
    int16_t	 sin;	/* sin(angle) */
    int16_t	 cos;	/* cos(angle) */
}tmcLib_PolarArg_s;


typedef struct
{
    uint16_t		r;		// amplitude
    tmcLib_PolarArg_s	t;		// argument
}tmcLib_Polar_s;


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
int16_t __ramfunc__ mcLib_Sine(uint16_t ang);
#else
int16_t mcLib_Sine(uint16_t ang);
#endif

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
int16_t __ramfunc__ mcLib_Cosine(uint16_t ang);
#else
int16_t mcLib_Cosine(uint16_t ang);
#endif

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
uint16_t  __ramfunc__ mcLib_InverseTangent(int16_t x, int16_t y);
#else
uint16_t mcLib_InverseTangent(int16_t x, int16_t y);
#endif


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
uint32_t  __ramfunc__ mcLib_SquareRoot(uint32_t number);
#else
uint32_t mcLib_SquareRoot( uint32_t number );
#endif

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
int16_t  __ramfunc__ mcLib_DetermineAdjSide(int16_t hypo, int16_t fcat);
#else
int16_t mcLib_DetermineAdjSide(int16_t hypo, int16_t fcat);
#endif 


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
int32_t __ramfunc__ mcLib_RightShift(int32_t operand, uint16_t shift );
#else
int32_t mcLib_RightShift(int32_t operand, uint16_t shift );
#endif

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
int32_t mcLib_MultShiftRight( const int16_t multiplicand1, const int16_t multiplicand2, const uint16_t shift );


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
void mcLib_FloatScaling( const float f32a, IQ * const iq, const uint8_t q);


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
void mcLib_IQMultiplication( IQ m1, IQ m2, IQ * result );

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
void __ramfunc__ mcLib_Cartesian2Polar(const tmcLib_Cartesian_s *xy, tmcLib_Polar_s *rt);
#else
void mcLib_Cartesian2Polar(const tmcLib_Cartesian_s *xy, tmcLib_Polar_s *rt);
#endif


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
void mcLib_FloatToIQ( float f32a, IQ * const result );

#endif // MC_MATH_H
