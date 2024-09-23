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

/*Define the Motor Type*/
#define LONG_HURST

/* Defining RAM_EXECUTE executes key motor control functions from RAM and thereby allowing faster execution at the expense of data memory
  Undefining RAM_EXECUTE executes key motor control functions from Flash and thereby reducing data memory consumption at the expense of time */
#define RAM_EXECUTE


/*******************************************************************************
Control Parameters
*******************************************************************************/
#define MC_FREQ_HZ      ( 48000000.0f ) /* CPU frequency */

#define PWM_FREQUENCY   (20)       /* in kHz   */

#define PWM_PERIOD      (2400U)

#define DEAD_TIME_EN
/*******************************************************************************
Motor definitions
*******************************************************************************/
/* motor and application related parameters */

#ifdef  LONG_HURST

#define MOTOR_POLE_PAIRS        (5U)  
#define MAX_MOTOR_SPEED         (3000U)
#define SPEED_KP_DEFAULT        (25U) 
#define SPEED_KI_DEFAULT        (12U)
#define CLOSED_SPEED_RPM_DEFAULT 2500U

#define DEFAULT_DUTY (uint16_t) ( 0.1 * PWM_PERIOD)
#define PI_BUF_INIT             (DEFAULT_DUTY << 15)

#endif





#define DEFAULT_SPEED_TARGET        ((DEFAULT_DUTY << 14)/PWM_PERIOD)

#define SPEED_MIN_LOW               (5U)   //  [Min speed in internal units range 0 - 10 Max; Typically less than 0% -10% of 2^14]

#define SPEED_MIN_TARGET            (uint16_t)(( 1 << 14) * SPEED_MIN_LOW / 100)

#define SPEED_MIN_LIMIT             (0U)

#define SPEED_MAX_LIMIT             (PWM_PERIOD)



/*BLDC zero-crossing macro*/
/*Define the number of comparator-pairs to be used*/
//#define AC_PAIR_NUM  1


#define OPENLOOP_END_SPEED_RPM  750U
#define OPENLOOP_START_SPEED_RPM 0U

#define MOTOR_RAMPUP_SPEED_PER_MS   (3U)  //  (1-7) Rate at which motor accelrerate at openloop
#define OPENLOOP_PWM_OFFSET      (0.0725f)   //  0.07 < OPENLOOP_PWM_OFFSET < 0.125
#define OPENLOOP_END_PWM_DUTY    (0.225f) //   OPENLOOP_PWM_OFFSET  < OPENLOOP_END_PWM_DUTY < 0.4 


#define OPENLOOP_ALIGN_PERIOD    (0.5f)  //unit second;
#define OPENLOOP_ALIGN_ENABLE    1U      //1: ENABLE; 0:DISABLE;

#define COMMUTE_COMPENSATION     80U    //80  //about 160us, unit T3 CLK 500KHz;
#define COMMUTATION_MIN_DELAY    2U     //unit T3 CLK 500KHz;
#define COMMUTATION_MAX_DELAY    2000U  //unit T3 CLK 500KHz;
#define BLANK_TIME               7500U  //156us; unit TC0 CLK 48MHz, 


/*******************************************************************************
 * Calculated macros
*******************************************************************************/
#define COUNT_10_MS                 (uint16_t)(PWM_FREQUENCY * 10)    //As PWM_FREQUENCY is given in KHz

/* Average Speed calculation factor */
#define MOTOR_SPEED_CALCFACTOR      6U   // (MOTOR_POLE_PAIRS * 6U)

#define MAX_POT_REF                 (4095U)

#endif // USERPARAMS_H
