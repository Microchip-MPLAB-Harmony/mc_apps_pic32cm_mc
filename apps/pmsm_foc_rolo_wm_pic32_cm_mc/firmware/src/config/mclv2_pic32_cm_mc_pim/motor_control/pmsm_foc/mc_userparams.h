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
* Copyright (C) 2021 Microchip Technology Inc. and its subsidiaries.
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

/*******************************************************************************
 * Application Configuration
*******************************************************************************/
#define ENABLE                      1u
#define DISABLE                     0u

/** 
 *  Defining DEVELOPER_MODE allows the programmer to monitor debug signals via X2C Scope 
 *  Note: X2C library has to be linked for the DEVELOPER_MODE to function                    
 */

#define DEVELOPER_MODE   1u


/**
 *  Defining Q_AXIS_STARTUP causes the open loop startup by injecting current in Q axis  
 *  Un-defining Q_AXIS_STARTUP cause the open loop startup by injecting current in D axis       
 */
#define Q_AXIS_STARTUP    1u

/**
 *  Defining OPEN_LOOP_FUNCTIONING forces the algorithm to operate in Rotor Angle Open 
 *  Loop mode i.e. angle reference is generated and not estimated       
 */
 #define OPEN_LOOP_FUNCTIONING  DISABLE

/**
 *  Defining FLUX_WEAKENING allows the PMSM motor to operate at speed higher than rated 
 *  speed                                                                                       
 */
 #define FLUX_WEAKENING DISABLE 

/** 
 * Defining TORQUE_MODE forces the algorithm to operate in Torque Mode i.e. no speed 
 *    control, the potentiometer input is used as torque reference                   
 */
#define TORQUE_MODE DISABLE
#if ( 1u == TORQUE_MODE ) 
#define TORQUE_INPUT_FROM_POTENTIOMETER ENABLE
#endif

/**
 *  Defining RAM_EXECUTE executes key motor control functions from RAM and thereby allowing
 *  faster execution at the expense of data memory.    
 *  Un-defining RAM_EXECUTE executes key motor control functions from Flash and thereby reducing
 *  data memory consumption at the expense of time
 *  
 * Note: Instruction breakpoint will not be asserted if that particular instruction
 * is being executed from RAM          
 */
#define RAM_EXECUTE

/**
 *  Defining USE_DIVAS uses the DIVAS peripheral for division and square root operatons         
 */
#define USE_DIVAS

/**
 *  Defining BIDIRECTION_CONTROL enables bi-directional control of the motor                            
 */ 
#define BIDIRECTION_CONTROL  ENABLE

/**
 *  Defining CONTROL_TO_PWM_1_2 sets the control to PWM ratio to 1:2                         
 */ 
#define CONTROL_TO_PWM_1_2   ENABLE


/*******************************************************************************
 * System Configuration
*******************************************************************************/
/**
 *  PWM frequency in Hz                            
 */ 
#define PWM_FREQUENCY   (10000U)

/**
 *  Current control Frequency                           
 */ 
#ifdef CONTROL_TO_PWM_1_2
#define CURRENT_CONTROL_FREQUENCY      ( 0.5f * PWM_FREQUENCY ) 
#else 
#define CURRENT_CONTROL_FREQUENCY      ( 1.0f * PWM_FREQUENCY ) 
#endif

/**
 * Current control sampling time                            
 */ 
#define CURRENT_CONTROL_SAMPLING_TIME    (float)(1.0f/(float)CURRENT_CONTROL_FREQUENCY)

/*******************************************************************************
 * Motor Parameters 
*******************************************************************************/
/**
 *  Winding Connection Type 
 */
#define WINDING_CONNECTION_TYPE  (STAR)

/**
  *  Check for motor isotropy                  
  */
#define NON_ISOTROPIC_MOTOR

/**
 * Rated electrical speed in RPM
 */
#define RATED_SPEED_IN_RPM   ((int16_t)2804)

/**
 * Maximum electrical speed  with Field Weakening enabled in RPM
 */
#define MAXIMUM_SPEED_IN_RPM    ((int16_t)3500)

/**
 * Minimum close loop electrical speed of the motor in RPM
 */
#define MINIMUM_SPEED_IN_RPM   (500) 

/**
 * Number of pole pairs  
 */
#define NUM_POLE_PAIRS  ((int16_t)5)  

 /**
  *  Stator resistance in Ohm                      
  */
#define MOTOR_PER_PHASE_RESISTANCE_IN_OHM   ((float)0.285)    

 /**
  *  Direct axis inductance in Henry                 
  */
#define DIRECT_AXIS_INDUCTANCE_IN_HENRY  ((float)0.00032)     

/**
  *  Quadrature axis inductance in Henry                 
  */
#define QUADRATURE_AXIS_INDUCTANCE_IN_HENRY  ((float)0.00032) 

 /** 
  * Back EMF Constant in Vpeak/kRPM                   
  */
#define MOTOR_BEMF_CONST_VOLTS_PER_KRPM_MECH  ((float)7.24)

 /** 
  * Air gap flux in Weber                   
  */
#define  AIR_GAP_FLUX    (float)(  60 * MOTOR_BEMF_CONST_VOLTS_PER_KRPM_MECH / ( 1.414 * 1000 * M_PI ))

 /** 
  * Maximum peak current of the motor in amperes                     
  */
#define MAXIMUM_MOTOR_CURRENT_IN_AMPERE   ((float)4)     


/*******************************************************************************
 * Board Parameters 
*******************************************************************************/
/**
 * Current sensing shunt resistance in Ohms
 */ 
#define CURRENT_SENSE_SHUNT_RESISTANCE_IN_OHM (float)0.025 

/**
 * Current sensing amplifier type
 */ 
#define INVERTING_CURRENT_GAIN_AMPLIFIER

/**
 * Current sensing amplifier gain
 */ 
#define CURRENT_SENSE_AMPLIFIER_GAIN (float)30 

/**
 * Voltage divider ratio for bus voltage measurement
 */
#define VOLTAGE_SENSE_DIVIDER_RATIO  (float)0.0625 

/*******************************************************************************
 * PI Controller Parameters  
*******************************************************************************/
/**
 * Speed PI Controller Parameters 
 */
#define SPEED_TS        (float)CURRENT_CONTROL_SAMPLING_TIME
#define SPEED_KP        (float)(0.001 * BASE_SPEED / BASE_CURRENT)
#define SPEED_KI        (float)(0.03 * CURRENT_CONTROL_SAMPLING_TIME * BASE_SPEED / BASE_CURRENT)
#define SPEED_YMAX      (float)MAXIMUM_MOTOR_CURRENT_IN_AMPERE / BASE_CURRENT

/**
 * Direct axis current PI Controller Parameters 
 */
#define D_CURRENT_TS    (float)CURRENT_CONTROL_SAMPLING_TIME
#define D_CURRENT_KP    (float)(0.71 * BASE_CURRENT / BASE_VOLTAGE)
#define D_CURRENT_KI    (float)(917 * CURRENT_CONTROL_SAMPLING_TIME * BASE_CURRENT / BASE_VOLTAGE)
#define D_CURRENT_YMAX  (float)(0.98)

/**
 * Quadrature axis current PI Controller Parameters 
 */
#define Q_CURRENT_TS     (float)CURRENT_CONTROL_SAMPLING_TIME
#define Q_CURRENT_KP     (float)(0.71 * BASE_CURRENT / BASE_VOLTAGE)
#define Q_CURRENT_KI     (float)(917 * CURRENT_CONTROL_SAMPLING_TIME * BASE_CURRENT / BASE_VOLTAGE)
#define Q_CURRENT_YMAX   (float)(0.98)



/*******************************************************************************
 * Torque Mode Parameters  
*******************************************************************************/
#define TORQUE_MODE_MAXIMUM_CURRENT   (float)( 0.2f )


/*******************************************************************************
 * Start-up Parameters  
*******************************************************************************/

/**
 * Forced alignment lock time 
 */ 
#define START_UP_LOCK_TIME_IN_SECOND   (2) 

/**
  * Start-up current in amperes                          
  */
#define START_UP_CURRENT_IN_AMPERE   (0.4) 


/**
 * Start-up ramp time
 */
#define START_UP_RAMP_TIME_IN_SECOND   (5)


/**
 * Start-up stabilization time
 */
#define START_UP_STAB_TIME_IN_SECOND   (2)


/**
 * Start-up transition speed 
 */
#define START_UP_TRANS_SPEED_IN_RPM    (500)

/*******************************************************************************
 * Q14 constants 
*******************************************************************************/
/* Base values */
#define BASE_VALUE                                   (16384)
#define SH_BASE_VALUE                                ( 14U )

/**
 * Base voltage 
 */ 
#define BASE_VOLTAGE   (float)( 3.30 / VOLTAGE_SENSE_DIVIDER_RATIO )

/**
 * Base current
 */ 
#define BASE_CURRENT  (float)( 1.65 / ( CURRENT_SENSE_AMPLIFIER_GAIN * CURRENT_SENSE_SHUNT_RESISTANCE_IN_OHM ))

/**
 * Base speed
 */ 
#if ( 1u == FLUX_WEAKENING )
#define BASE_SPEED  (float)(2.0f * FLOAT_PI * ( (float)NUM_POLE_PAIRS * (float)MAXIMUM_SPEED_IN_RPM / 60.0f  ))
#else 
#define BASE_SPEED  (float)(2.0f * FLOAT_PI * ( (float)NUM_POLE_PAIRS * (float)RATED_SPEED_IN_RPM / 60.0f  ))
#endif 

/**
 * Base impedence = Base voltage / Base current
 */  
#define BASE_IMPEDENCE    (float)(BASE_VOLTAGE/BASE_CURRENT)

/**
 * Base inductance = Base Impedence / Base speed
 */ 
#define BASE_INDUCTANCE   (float)(BASE_IMPEDENCE/BASE_SPEED)

/**
 * 
 */ 
#define K_TIME    (float)CURRENT_CONTROL_FREQUENCY

/**
 *  Real voltage to scaled voltage conversion factor
 */ 
#define K_VOLTAGE  (float)(BASE_VALUE_FL / BASE_VOLTAGE)  

/**
 * Real current to scaled current conversion factor
 */ 
#define K_CURRENT  (float)(BASE_VALUE_FL / BASE_CURRENT)

/**
 * Real impedence to scaled impedence conversion factor
 */ 
#define K_IMPEDENCE  (float)BASE_CURRENT / BASE_VOLTAGE 


/**
 * Real inductance to scaled inductance conversion factor
 */ 
#define K_INDUCTANCE  (float)BASE_CURRENT / ( BASE_VOLTAGE * BASE_SPEED )


/**
 * Real speed to scaled speed conversion factor
 */ 
#define K_SPEED  (float)(BASE_VALUE_FL / BASE_SPEED)  

/**
 * Real speed to scaled speed conversion factor for angle calculation
 */ 
#define K_SPEED_L  (uint16_t)(BASE_SPEED * (32768.0f / FLOAT_PI) / K_TIME)

/**
 * ADC count to scaled voltage conversion factor 
 */ 
#define KAD_VOL  (uint16_t)(4)  

/**
 *  ADC count to scaled voltage conversion shift
 */ 
#define SH_KAD_VOL  (  0 )

/**
 * ADC count to scaled current conversion factor 
 */ 
#ifdef INVERTING_CURRENT_GAIN_AMPLIFIER
#define KAD_CUR  (uint16_t)(  -8 )  
#else   
#define KAD_CUR  (uint16_t)( 8 )  
#endif  

/**
 * ADC count to scaled voltage conversion shift 
 */  
#define SH_KAD_CUR  (  0 )

#endif // USERPARAMS_H

