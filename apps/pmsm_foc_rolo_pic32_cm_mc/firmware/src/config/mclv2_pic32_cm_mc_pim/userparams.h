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

/*******************************************************************************
 * Application Configuration
*******************************************************************************/

/** 
 *  Defining DEVELOPER_MODE allows the programmer to monitor debug signals via X2C Scope 
 *  Note: X2C library has to be linked for the DEVELOPER_MODE to function                    
 */

#define DEVELOPER_MODE


/**
 *  Defining Q_AXIS_STARTUP causes the open loop startup by injecting current in Q axis  
 *  Un-defining Q_AXIS_STARTUP cause the open loop startup by injecting current in D axis       
 */
#define Q_AXIS_STARTUP

/**
 *  Defining OPEN_LOOP_FUNCTIONING forces the algorithm to operate in Rotor Angle Open 
 *  Loop mode i.e. angle reference is generated and not estimated       
 */
#undef OPEN_LOOP_FUNCTIONING

/**
 *  Defining ENABLE_FLUX_WEAKENING allows the PMSM motor to operate at speed higher than rated 
 *  speed                                                                                       
 */
#define ENABLE_FLUX_WEAKENING 

/** 
 * Defining TORQUE_MODE forces the algorithm to operate in Torque Mode i.e. no speed 
 *    control, the potentiometer input is used as torque reference                   
 */
#undef TORQUE_MODE
#ifdef TORQUE_MODE
#undef TORQUE_INPUT_FROM_POTENTIOMETER
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
 *  Defining CURPI_TUN enables current PI tuning mode in which a step current reference is
 *  generated to observe step response of the current controller                               
 */ 
#undef CURPI_TUN 
#ifdef CURPI_TUN
#define CUR_STEP_AMP    ( 1.0 )     
#define CUR_STEP_VAL    ((int16_t)(CUR_STEP_AMP * BASE_VALUE / BASE_CURRENT))
#define CUR_STEP_TIM    ( 0.5 )     
#define CPT_CNT_VAL     ((uint16_t)(CUR_STEP_TIM * CURRENT_CONTROL_FREQUENCY ))
#endif

/**
 *  Defining BIDIRECTION_CONTROL enables bi-directional control of the motor                            
 */ 
#define BIDIRECTION_CONTROL

/**
 *  Defining CONTROL_TO_PWM_1_2 sets the control to PWM ratio to 1:2                         
 */ 
#define CONTROL_TO_PWM_1_2



/*******************************************************************************
 * Harmony Configuration
*******************************************************************************/
/**
 *  PWM Timer ticks                             
 */ 
#define PWM_HPER_TICKS     (  2400U       )  

/**
 *  Clock Frequency                           
 */ 
#define MC_FREQ_HZ      (  48000000.0f ) 

/**
 *  Dead time timer ticks                          
 */ 
#define DEADT_TICKS   (   48U        )	

/*******************************************************************************
 * System Configuration
*******************************************************************************/
/**
 *  PWM frequency in Hz                            
 */ 
#define PWM_FREQUENCY       ( 0.5f * MC_FREQ_HZ / (float)PWM_HPER_TICKS )

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
 *  Select motor to use default Motor parameters  
 */
#define LONG_HURST                                    (1U)

/**
 *  PMSM Parameters  
 */
#if( 1U == LONG_HURST )

/**
  *  Check for motor isotropy                  
  */
#define NON_ISOTROPIC_MOTOR

/**
 * Rated electrical speed in RPM
 */
#define RATED_SPEED_IN_RPM      (   3000  ) 

/**
 * Maximum electrical speed  with Field Weakening enabled in RPM
 */
#define MAXIMUM_SPEED_IN_RPM    (   3400  )

/**
 * Minimum close loop electrical speed of the motor in RPM
 */
#define MINIMUM_SPEED_IN_RPM    (   500   )   

/**
 * Number of pole pairs  
 */
#define NUM_POLE_PAIRS    (     5   )  

 /**
  *  Stator resistance in Ohm                      
  */
#define MOTOR_PER_PHASE_RESISTANCE_IN_OHM           (float)(   2.10  )    

 /**
  *  Direct axis inductance in Henry                 
  */
#define DIRECT_AXIS_INDUCTANCE_IN_HENRY        (float)( 0.00192 )     

/**
  *  Quadrature axis inductance in Henry                 
  */
#define QUADRATURE_AXIS_INDUCTANCE_IN_HENRY        (float)( 0.00192 )    

 /** 
  * Back EMF Constant in Vpeak/kRPM                   
  */
#define MOTOR_BEMF_CONST_VOLTS_PER_KRPM_MECH        (float)(   6.2   )

 /** 
  * Air gap flux in Weber                   
  */
#define  AIR_GAP_FLUX    (float)(  60 * MOTOR_BEMF_CONST_VOLTS_PER_KRPM_MECH / ( 1.414 * 1000 * M_PI ))

 /** 
  * Maximum peak current of the motor in amperes                     
  */
#define MAXIMUM_MOTOR_CURRENT_IN_AMPERE        (float)(   2.0   )      


#endif

/*******************************************************************************
 * Board Parameters 
*******************************************************************************/

/* Select the Board Type */
#define   MCLV2 

/* dsPICDEM MCLV-2 Board related parameters */
#ifdef MCLV2
/*_______________________ VOLTAGE SENSING __________________________*/  
#define VOLTAGE_SENSE_TOP_RESISTANCE_IN_KOHM         (float)30.0f
#define VOLTAGE_SENSE_BOTTOM_RESISTANCE_IN_KOHM      (float)2.0f

/* ______________________ CURRENT SENSING __________________________*/
#define INVERTING_TYPE_CURRENT_SENSE_AMPLIFIER  
#define CURRENT_SENSE_AMPLIFIER_GAIN                 ( 30.000 )
#define CURRENT_SENSE_SHUNT_RESISTANCE_IN_OHM        (  0.025 )
#endif


#define VOLTAGE_SENSE_DIVIDER_RATIO                  (float)( VOLTAGE_SENSE_BOTTOM_RESISTANCE_IN_KOHM / ( VOLTAGE_SENSE_TOP_RESISTANCE_IN_KOHM + VOLTAGE_SENSE_BOTTOM_RESISTANCE_IN_KOHM))
#define MAXIMUM_MEASURABLE_VOLTAGE_IN_VOLT           (float)( 3.30  / VOLTAGE_SENSE_DIVIDER_RATIO )
#define MAXIMUM_MEASURABLE_CURRENT_IN_AMPERE         (float)( 1.65 / ( CURRENT_SENSE_AMPLIFIER_GAIN * CURRENT_SENSE_SHUNT_RESISTANCE_IN_OHM ))


/*******************************************************************************
 * PI Controller Parameters  
*******************************************************************************/
/**
 * Speed PI Controller Parameters 
 */
#define SPEED_TS       (float)CURRENT_CONTROL_SAMPLING_TIME
#define SPEED_KP        (float)0.5f
#define SPEED_KI        (float)0.005f
#define SPEED_KC        (float)0.999f
#define SPEED_YMAX  (float)( MAXIMUM_MOTOR_CURRENT_IN_AMPERE / BASE_CURRENT )

/**
 * Direct axis current PI Controller Parameters 
 */
#define D_CURRENT_TS       (float)CURRENT_CONTROL_SAMPLING_TIME
#define D_CURRENT_KP        (float)0.02f
#define D_CURRENT_KI        (float)0.8f
#define D_CURRENT_KC        (float)0.999f
#define D_CURRENT_YMAX  (float)0.98f

/**
 * Quadrature axis current PI Controller Parameters 
 */
#define Q_CURRENT_TS      (float)CURRENT_CONTROL_SAMPLING_TIME
#define Q_CURRENT_KP       (float)0.02f
#define Q_CURRENT_KI       (float)0.8f
#define Q_CURRENT_KC       (float)0.999f
#define Q_CURRENT_YMAX  (float)0.98f



/*******************************************************************************
 * Start-up Parameters  
*******************************************************************************/

/**
 * ToDO: Development under progress 
 */
#define ENABLE_START_UP_COMPENSATION   

#ifdef ENABLE_START_UP_COMPENSATION
#define INITIAL_INTEGRAL_VALUE   (int32_t)2000
#endif

/**
  * Start-up current in amperes                          
  */
#define START_UP_CURRENT_IN_AMPERE      (float)(   0.2   ) 


/**
 * Start-up ramp time
 */
#define START_UP_RAMP_TIME_IN_SECOND      (float)(   5   ) 


/**
 * Start-up transition speed 
 */
#define START_UP_TRANS_SPEED_IN_RPM     (int16_t)500

/*******************************************************************************
 * Q14 constants 
*******************************************************************************/
/* Base values */
#define BASE_VALUE                                   (16384)
#define SH_BASE_VALUE                                ( 14U )

/* Base Quantities */
#define BASE_VOLTAGE                                 (float) MAXIMUM_MEASURABLE_VOLTAGE_IN_VOLT          
#define BASE_CURRENT                                 (float) MAXIMUM_MEASURABLE_CURRENT_IN_AMPERE       
#ifdef ENABLE_FLUX_WEAKENING
#define BASE_SPEED                                   (float)(2.0f * FLOAT_PI * ( (float)NUM_POLE_PAIRS * (float)MAXIMUM_SPEED_IN_RPM / 60.0f  ))
#else 
#define BASE_SPEED                                   (float)(2.0f * FLOAT_PI * ( (float)NUM_POLE_PAIRS * (float)RATED_SPEED_IN_RPM / 60.0f  ))
#endif 
  
#define BASE_IMPEDENCE                               (float)(BASE_VOLTAGE/BASE_CURRENT)
#define BASE_INDUCTANCE                              (float)(BASE_IMPEDENCE/BASE_SPEED)

/* Conversion Factors */
#define K_TIME                                       (float)CURRENT_CONTROL_FREQUENCY
#define K_VOLTAGE                                    (float)(BASE_VALUE_FL / BASE_VOLTAGE)  
#define K_CURRENT                                    (float)(BASE_VALUE_FL / BASE_CURRENT)
#define K_IMPEDENCE                                  (float)BASE_CURRENT / BASE_VOLTAGE 
#define K_SPEED                                      (float)(BASE_VALUE_FL / BASE_SPEED)  
#define K_SPEED_L                                    (uint16_t)(BASE_SPEED * (32768.0f / FLOAT_PI) / K_TIME)
#define KAD_VOL                                      (uint16_t)(  4 )  
#ifndef INVERTING_TYPE_CURRENT_SENSE_AMPLIFIER
#define KAD_CUR                                      (uint16_t)(  8 )  
#else   
#define KAD_CUR                                      (uint16_t)( -8 )  
#endif  
#define SH_KAD_CUR                                   (  0 )
#define SH_KAD_VOL                                   (  0 )
	

#endif // USERPARAMS_H

