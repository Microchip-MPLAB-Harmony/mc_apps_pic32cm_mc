/*******************************************************************************
  System Definitions

  File Name:
    mc_app.h

  Summary:
    Header file which shares global variables and function prototypes.

  Description:
    This file contains the global variables and function prototypes for a motor control project.

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

#ifndef MC_APP_H
#define MC_APP_H

#include "userparams.h"
#include "q14_generic_mcLib.h"


#define Q_NUM 14


/* Motor states */
#define MC_STATE_STOP              (0u)
#define MC_STATE_OPENLOOP          (1u)
#define MC_STATE_START_BEMF_MEAS   (2u)
#define MC_STATE_BEMF_TRANSITION   (3u)
#define MC_STATE_BEMF_COMMUTATION  (4u)
#define MC_STATE_SPEEDLOOP_INITIAL (5u)
#define MC_STATE_SPEEDLOOP_CLOSED  (6u)

#define MOTOR_BEMF_COMMUTATION_WAIT_CNT      6   

/*******************************************************************************
Public typedefs
*******************************************************************************/


typedef enum
{
    NO_START_CMD,
    STOP_CMD,  
    COMMUTATION_FAULT_STOP,
    STALL_FAULT_STOP, 
    OC_FAULT_STOP
} stop_source_t;


typedef struct
{
    uint16_t speed_ref_pot;
    uint16_t set_speed_target;
    uint16_t set_speed_target_rpm;
    uint16_t speed_reference_target;
    uint16_t speed_reference_rpm;
    uint32_t avgtimestorage;
    uint32_t avgcycletime;
    uint16_t actual_speed;
    uint16_t actual_speed_target;
    uint16_t avgctr;  
    uint16_t motor_current;    
    uint32_t speed_constant;
    uint8_t  speed_pi_enable;       
    
}motor_bc_params_t;

typedef struct
{
    uint16_t openloop_end_speed;
    uint16_t openloop_start_speed;
    uint16_t openloop_ramp_speed;
    
    uint16_t openloop_pwm_offset;
    uint16_t openloop_pwm_factor;
    uint16_t openloop_end_pwm;
    
    uint16_t openloop_rotor_angle;
    uint16_t openloop_rotor_angle_delta;
    uint16_t RPMtoAngle_factor;
    
    uint16_t Enbale_Align;
    uint16_t AlignCounter;   
}motor_sensorless_params_t;

typedef struct
{
    stop_source_t motor_stop_source;
    uint8_t  state_run; 
    uint8_t  switch_state;
    uint8_t  direction;
    uint8_t  direction_offset;
    uint8_t  var_time_10ms;
    uint8_t  var_cnt_10ms;
    uint8_t  motor_control_state;
    uint8_t  mc_state_counter;
    uint8_t  wrong_commutation_counter;      
    uint16_t stall_counter;
    uint16_t commutation_time;
    
}motor_state_params_t;


typedef enum
{
    phase_U,
    phase_V,          
    phase_W
} phase_t;

typedef struct 
{
    uint8_t curpattern;   
    uint8_t nextpattern;
    uint8_t patt_enable;
    uint8_t patt_value;
    uint16_t pattern_commutation;
    uint8_t curhall1;
    uint8_t curhall2;
    uint8_t curhall3;  

}motor_hall_params_t;

typedef struct 
{
    uint8_t curBEMF_A;
    uint8_t curBEMF_B;
    uint8_t curBEMF_C;
    uint8_t curBEMFpattern;
    uint8_t curBEMFpattern_prv;
    uint8_t curpattern_openmode; 
    uint8_t nextpattern_openmode; 
    uint8_t nextBEMFpattern;
    uint8_t curstate;    
    phase_t undriven_phase;  
    uint16_t Commutation_Delay;
}motor_BEMF_params_t;

extern picontrol_type  speedpi;
/*******************************************************************************
Public variables definition
*******************************************************************************/

//extern stop_source_t motor_stop_source; 

extern uint8_t var_time_10ms;


/*******************************************************************************
Private functions prototypes
*******************************************************************************/


/*******************************************************************************
Public functions prototypes
*******************************************************************************/


/******************************************************************************
Function:     MCAPP_MotorControlVarsInit
Description:  motor control variable initialization
Input:        nothing (uses some global variables)
Output:       nothing (modifies some global variables)
Note:         to be called once before starting the control functions              
******************************************************************************/
void MCAPP_MotorControlVarsInit(void);



void Motor_Start(void);
void Motor_Stop(void);
void MCAPP_Start(void);

uint16_t speed_ramp(uint16_t Ref, uint16_t ramp_rate, uint16_t ref_temp);
void SixStep_OpenLoop_Commutation(void);
void BEMF_Period_Measure(void);
void AC_blank_end(TC_TIMER_STATUS status, uintptr_t context);
void Read_UndrivenPhase(void);

#endif // MC_APP_H