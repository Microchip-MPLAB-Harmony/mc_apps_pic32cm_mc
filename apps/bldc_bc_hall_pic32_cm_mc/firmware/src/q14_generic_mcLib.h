/*******************************************************************************
  System Definitions

  File Name:
    q14_generic_mcLib.h

  Summary:
    Header file which contains variables and function prototypes for Motor Control.
 
  Description:
    This file contains variables and function prototypes which are generally used in Motor Control.
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

#ifndef Q14_GENERIC_MCLIB_H
#define Q14_GENERIC_MCLIB_H

#include <stdint.h>
#include <sys/attribs.h>
#include "userparams.h"
/******************************************************************************
Macro definitions and typedefs
******************************************************************************/

typedef struct picontrol_type_t {
	/* kp Gain */
	int32_t kp;
	/* ki Gain */
	int32_t ki;
	/* Old error value */
	int32_t error;
	/*old output value */
	int32_t outputvalue;
	/* Max Limit of output */
	int32_t maxlimit;
	/* Min Limit of output value */
	int32_t minlimit;
    /* integrator buffer */
    uint32_t integratorBuf;
} picontrol_type;

/******************************************************************************
Functions prototypes
******************************************************************************/

#ifdef RAM_EXECUTE
uint16_t __ramfunc__ pi_lib_calculate(picontrol_type* piPtr);
#else
uint16_t pi_lib_calculate(picontrol_type* piPtr);
#endif

//#ifndef   STATIC_FORCEINLINE
//  #define STATIC_FORCEINLINE                   __attribute__((always_inline)) static inline
//#endif
/*! \brief Right shift  for 32-bit signed integer
 *
 * Details
 * Right shift  for 32-bit signed integer
 *
 * @param[in]:
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
static inline int32_t mcUtils_RightShiftS32(int32_t operand, uint16_t shift )
{
#if defined ENABLE_SIGN_INTEGER_SHIFT
    /* MISRAC 2012 deviation block start */
    /* MISRA C-2012 Rule 10.1 deviated in this function. Deviation record ID -  H3_MISRAC_2012_R_2_2_DR_1  */
    return ( operand >> shift );
    /* MISRAC 2012 deviation block end */
#else
    int32_t result;
    uint32_t u32a;

    if( 0 > operand )
    {
        u32a = ((uint32_t)(-operand ) >> shift );
        result = -(int32_t)u32a;
    }
    else
    {
        u32a = ((uint32_t)operand >> shift);
        result = (int32_t)u32a;
    }
    return result;
#endif
}

/*! \brief Left shift  for 32-bit signed integer
 *
 * Details
 * Left shift  for 32-bit signed integer
 *
 * @param[in]:
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
static inline int32_t mcUtils_LeftShiftS32(int32_t operand, uint16_t shift )
{
#if defined ENABLE_SIGN_INTEGER_SHIFT
    /* MISRAC 2012 deviation block start */
    /* MISRA C-2012 Rule 10.1 deviated in this function. Deviation record ID -  H3_MISRAC_2012_R_2_2_DR_1  */
    return ( operand << shift );
    /* MISRAC 2012 deviation block end */
#else
    int32_t result;
    uint32_t u32a;

    if(0 > operand )
    {
        u32a = ((uint32_t)(-operand ) << shift );
        result = -(int32_t)u32a;
    }
    else
    {
        u32a = ((uint32_t)operand << shift);
        result = (int32_t)u32a;
    }
    return (result);
#endif
}

/*! \brief Right shift  for 16-bit signed short
 *
 * Details
 * Right shift  for 16-bit signed short
 *
 * @param[in]:
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
static inline int16_t mcUtils_RightShiftS16(int16_t operand, uint16_t shift )
{
#if defined ENABLE_SIGN_INTEGER_SHIFT
    /* MISRAC 2012 deviation block start */
    /* MISRA C-2012 Rule 10.1 deviated in this function. Deviation record ID -  H3_MISRAC_2012_R_2_2_DR_1  */
    return ( operand >> shift );
    /* MISRAC 2012 deviation block end */
#else
    int16_t result;
    uint16_t u16a;

    if( 0 > operand )
    {
        u16a = ((uint16_t)(-operand ) >> shift );
        result = -(int16_t)u16a;
    }
    else
    {
        u16a = ((uint16_t)operand >> shift);
        result = (int16_t)u16a;
    }
    return (result);
#endif
}

/*! \brief Left shift  for 16-bit signed short
 *
 * Details
 * Left shift  for 16-bit signed short
 *
 * @param[in]:
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
static inline int16_t mcUtils_LeftShiftS16(int16_t operand, uint16_t shift )
{
#if defined ENABLE_SIGN_INTEGER_SHIFT
    /* MISRAC 2012 deviation block start */
    /* MISRA C-2012 Rule 10.1 deviated in this function. Deviation record ID -  H3_MISRAC_2012_R_2_2_DR_1  */
    return ( operand << shift );
    /* MISRAC 2012 deviation block end */
#else
    int16_t result;
    uint16_t u16a;

    if( 0 > operand )
    {
        u16a = ((uint16_t)(-operand ) << shift );
        result = -(int16_t)u16a;
    }
    else
    {
        u16a = ((uint16_t)operand << shift);
        result = (int16_t)u16a;
    }
    return (result);
#endif
}

/*! \brief Left shift  for 16-bit signed short
 *
 * Details
 * Left shift  for 16-bit signed short
 *
 * @param[in]:
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
static inline int16_t mcUtils_MultAndRightShiftS16(int16_t operand1, int16_t operand2, uint16_t shift )
{
#if defined ENABLE_SIGN_INTEGER_SHIFT
    /* MISRAC 2012 deviation block start */
    /* MISRA C-2012 Rule 10.1 deviated in this function. Deviation record ID -  H3_MISRAC_2012_R_2_2_DR_1  */
    return (int16_t)(( (int32_t)operand1 * (int32_t)operand2)  >> shift );
    /* MISRAC 2012 deviation block end */
#else
    return (int16_t)mcUtils_RightShiftS32((int32_t)((int32_t)operand1 * (int32_t)operand2 ), shift );
#endif
}

/* END OF CONTROL FUNCTIONS *************************************************/


#endif // Q14_GENERIC_MCLIB_H

