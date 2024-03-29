/*******************************************************************************
  System Definitions

  File Name:
    userparams.h

  Summary:
    Header file which defines Motor Specific and Board Specific constants 

  Description:
    This file contains the motor and board specific constants. It also defines
 * switches which allows algorithm to be run in debug modes like Open Loop Mode,
 * Torque mode, etc. 

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

#ifndef USERPARAMS_H 
#define USERPARAMS_H

typedef float   float32_t;
/*Define the Motor Type*/
#define AC_IM_1    

/*******************************************************************************
Macro definitions
*******************************************************************************/
#define MC_FREQ_HZ      ( 48000000.0f ) /* PWM peripheral frequency */

#define PWM_FREQUENCY 20U       /* in kHz   */

#define PWM_HPER_TICKS       ( 1200U )  /* 4000 ticks, total period 8000 ticks @48MHz -> 167us */
#define PWM_HPER_TICKS_FL       ( 1200.0f )  /* 4000 ticks, total period 8000 ticks @48MHz -> 167us */
#define HALF_PWM_HPER_TICKS  (uint16_t)(PWM_HPER_TICKS >> 1U)
/* 1200 ticks , total period 2400 ticks -> 50 micro seconds */

/* motor and application related parameters */
/* Note: only one motor has to be selected! */

#ifdef  AC_IM_1                     /* ACIM Example motor */

#define MOTOR_VOLTAGE           (230)                   /* Motor Voltage [Volts] */
#define MAX_MOTOR_SPEED         (1800U)                  /* maximum motor speed [RPM] */
#define NUMBER_OF_POLES         (4U)                     /* number of poles  */
#define MAX_SPEED_SCALED        (16384U)                /* Max speed in internal Units */
#define VF_CONSTANT             ((PWM_HPER_TICKS << 12) /MAX_SPEED_SCALED)      /* (PWM_HPER_TICKS/MAX_SPEED_SCALED) * 2^12 */
#define VF_OFFSET               (float32_t)(0.09f * PWM_HPER_TICKS_FL)/* Voltage offset at starting zero freq; Approx (0.09 * PWM_HPER_TICKS)  */
#define START_SPEED_DEFAULT     (200U)                  /* Default startup speed */
#define ACC_RAMP                (1)                     /* acceleration ramp count in internal unit */
#define DEC_RAMP                (1)                     /* deceleration ramp count in internal unit */
#define SPEED_FILTER_COEFF      (10)                    /* Speed filter coefficient range [1-16]    */

#endif  /* ifdef AC_IM_1 */

#ifdef  AC_IM_2                     /* ACIM Example motor */

#define MOTOR_VOLTAGE           (230)                   /* Motor Voltage [Volts] */
#define MAX_MOTOR_SPEED         (3600)                  /* maximum motor speed [RPM] */
#define NUMBER_OF_POLES         (2)                     /* number of poles  */
#define MAX_SPEED_SCALED        (16384U)                /* Max speed in internal Units */
#define VF_CONSTANT             ((PWM_HPER_TICKS << 12)/MAX_SPEED_SCALED)       /* (PWM_HPER_TICKS/MAX_SPEED_SCALED) * 2^12 */
#define VF_OFFSET               (0.09f * PWM_HPER_TICKS)/* Voltage offset at starting zero freq; Approx (0.09 * PWM_HPER_TICKS)  */
#define START_SPEED_DEFAULT     (200U)                  /* Default startup speed */
#define ACC_RAMP                (1)                     /* acceleration ramp count in internal unit */
#define DEC_RAMP                (1)                     /* deceleration ramp count in internal unit */
#define SPEED_FILTER_COEFF      (10)                    /* Speed filter coefficient range [1-16]    */

#endif  /* ifdef AC_IM_1 */


#endif // USERPARAMS_H

