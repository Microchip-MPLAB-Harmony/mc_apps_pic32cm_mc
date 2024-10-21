/*******************************************************************************
 Motor Control Application Source file

  Company:
    Microchip Technology Inc.

  File Name:
    mc_app.c

  Summary:
    This file contains all the functions related to motor control application

  Description:
    This file contains implementation of the application state machine

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
#include "q14_generic_mcLib.h"
#include "device.h"

#include "definitions.h"
#include "mc_app.h"
#include <sys/attribs.h>
#include "userparams.h"
#include "X2Cscope.h"

/*******************************************************************************
Variables
*******************************************************************************/
motor_bc_params_t       Motor_BCParams;
motor_hall_params_t     Motor_HallParams;
motor_state_params_t    Motor_StateParams;
picontrol_type          Motor_Speed_PIParams;

motor_sensorless_params_t Motor_SensorlessParams;
motor_BEMF_params_t       Motor_BEMFParams;

/* Hall Pattern
 first 8 entries are for clockwise direction and later 8 for anti-clockwise direction
 */
const uint16_t HALL_ARRAY[16] = { 0, 5, 3, 1, 6, 4, 2, 0, 0, 3, 6, 2, 5, 1, 4, 0 };

/* Commutation Pattern as per Hall Pattern
 first 8 entries are for clockwise direction and later 8 for anti-clockwise direction
 */
#ifndef DEAD_TIME_EN
const uint16_t COMMUTATION_ARRAY[16] = {
    0,
    /* to achieve B+ C-, put the following in Pattern register H1H2H3: 001 */
    0x4075,
    /* to achieve A+ B-, put the following in Pattern register H1H2H3: 010 */
    0x2076,
    /* to achieve A+ C-, put the following in Pattern register H1H2H3: 011 */
    0x4076,
    /* to achieve C+ A-, put the following in Pattern register H1H2H3: 100 */
    0x1073,
    /* to achieve B+ A-, put the following in Pattern register H1H2H3: 101 */
    0x1075,
    /* to achieve C+ B-, put the following in Pattern register H1H2H3: 110 */
    0x2073,
    /* Not a valid pattern */
    0,
    0,
    0x2073,
    0x1075,
    0x1073,
    0x4076,
    0x2076,
    0x4075,
    0
};
#else
const uint16_t COMMUTATION_ARRAY[16] = {

    0,
    /* to achieve B+ C-, put the following in Pattern register H1H2H3: 001 */
    0x4075,
    /* to achieve A+ B-, put the following in Pattern register H1H2H3: 010 */
    0x2076,
    /* to achieve A+ C-, put the following in Pattern register H1H2H3: 011 */
    0x4076,
    /* to achieve C+ A-, put the following in Pattern register H1H2H3: 100 */
    0x1073,
    /* to achieve B+ A-, put the following in Pattern register H1H2H3: 101 */
    0x1075,
    /* to achieve C+ B-, put the following in Pattern register H1H2H3: 110 */
    0x2073,
    /* Not a valid pattern */
    0,
    0,
    0x2073,
    0x1075,
    0x1073,
    0x4076,
    0x2076,
    0x4075,
    0
};
#endif

/*******************************************************************************
Function Prototypes
*******************************************************************************/
#ifdef RAM_EXECUTE
void __ramfunc__ ADC_ISR(ADC_STATUS status, uintptr_t context);
void __ramfunc__ OC_FAULT_ISR(uintptr_t context);
void __ramfunc__ TC4_1ms_ISR(TC_TIMER_STATUS status, uintptr_t context);
void __ramfunc__ Hall_UpdateCommutation_ISR(void);
void __ramfunc__ BEMF_Commutation(TC_TIMER_STATUS status, uintptr_t context);
void __ramfunc__ ac_callBack(uint8_t int_flag, uintptr_t ac_context);
#else
void ADC_ISR(ADC_STATUS status, uintptr_t context);
void OC_FAULT_ISR(uintptr_t context);
void TC4_1ms_ISR(TC_TIMER_STATUS status, uintptr_t context);
void Hall_UpdateCommutation_ISR(void);
void BEMF_Commutation(TC_TIMER_STATUS status, uintptr_t context);
void ac_callBack(uint8_t int_flag, uintptr_t ac_context);
#endif
/*******************************************************************************
Functions
*******************************************************************************/

/******************************************************************************
Function:     MCAPP_MotorControlVarsInit
Description:  motor control variable initialization
Input:        nothing (uses some global variables)
Output:       nothing (modifies some global variables)
Note:         to be called once before starting the control functions
******************************************************************************/
void MCAPP_MotorControlVarsInit(void)
{
    Motor_StateParams.state_run = 0;
    Motor_StateParams.motor_stop_source = NO_START_CMD;
    Motor_StateParams.direction_offset = 0;
    Motor_StateParams.switch_state = 0;
    Motor_StateParams.direction = 0;

    Motor_HallParams.curpattern = 1;

    Motor_Speed_PIParams.error = 0;
    Motor_Speed_PIParams.kp = (int32_t)SPEED_KP_DEFAULT;
    Motor_Speed_PIParams.ki = (int32_t)SPEED_KI_DEFAULT;
    Motor_Speed_PIParams.maxlimit = SPEED_MAX_LIMIT;
    Motor_Speed_PIParams.minlimit = SPEED_MIN_LIMIT;
    Motor_Speed_PIParams.outputvalue = 0;
    Motor_Speed_PIParams.integratorBuf = 0;
    Motor_BCParams.set_speed_target_rpm = CLOSED_SPEED_RPM_DEFAULT;
    Motor_BCParams.speed_constant = (TC3_TimerFrequencyGet() * 10) / MOTOR_POLE_PAIRS;
    Motor_BCParams.speed_pi_enable = 0;

    Motor_SensorlessParams.openloop_end_speed = ((uint32_t)OPENLOOP_END_SPEED_RPM<<Q_NUM)/MAX_MOTOR_SPEED;
    Motor_SensorlessParams.openloop_start_speed = ((uint32_t)OPENLOOP_START_SPEED_RPM<<Q_NUM)/MAX_MOTOR_SPEED;
    Motor_SensorlessParams.openloop_ramp_speed = Motor_SensorlessParams.openloop_start_speed;

    Motor_SensorlessParams.openloop_pwm_offset = OPENLOOP_PWM_OFFSET*PWM_PERIOD;
    Motor_SensorlessParams.openloop_pwm_factor = ((uint32_t)((OPENLOOP_END_PWM_DUTY)*PWM_PERIOD)<<Q_NUM)/OPENLOOP_END_SPEED_RPM;
    Motor_SensorlessParams.openloop_end_pwm = (uint16_t)(OPENLOOP_END_PWM_DUTY*PWM_PERIOD);

    Motor_SensorlessParams.RPMtoAngle_factor = 32768.0*MOTOR_POLE_PAIRS*6.0/60.0/(PWM_FREQUENCY*1000.0)*(float)(1<<Q_NUM);   // six step commutation, so x6; 32768 stand for 1 revolution;
    Motor_SensorlessParams.openloop_rotor_angle = 0;

    Motor_SensorlessParams.AlignCounter = OPENLOOP_ALIGN_PERIOD*1000;
    Motor_SensorlessParams.Enbale_Align = OPENLOOP_ALIGN_ENABLE;
}

/******************************************************************************
Function:     MCAPP_Start
Description:  motor control start function
Input:        nothing (uses some global variables)
Output:       nothing (modifies some global variables)
Note:         called when start button is pressed
******************************************************************************/
void MCAPP_Start(void)
{
    /* ADC result ready interrupt handler to read Ishunt and Potentiometer value */
    ADC0_CallbackRegister((ADC_CALLBACK) ADC_ISR, (uintptr_t)NULL);

    /* Fault interrupt handler */
    EIC_CallbackRegister ((EIC_PIN)EIC_PIN_15, (EIC_CALLBACK) OC_FAULT_ISR,(uintptr_t)NULL);

    /* 1ms timer interrupt handler for speed calculation */
    TC4_TimerCallbackRegister(TC4_1ms_ISR, (uintptr_t)NULL);

    AC_CallbackRegister(ac_callBack,(uintptr_t)NULL);
    TC2_TimerCallbackRegister(BEMF_Commutation, (uintptr_t)NULL);
    TC0_TimerCallbackRegister(AC_blank_end, (uintptr_t)NULL);

    MCAPP_MotorControlVarsInit();

    /* Enable ADC0 and thus it's slave ADC1 */
    ADC0_Enable();

    TCC0_PWMStart();

    /* Disable all PWM outputs */
    TCC0_PWMPatternSet(0x77, 0x00);

    /* Start 1 mS timer */
    TC4_TimerStart();
}


/******************************************************************************
Function:     Motor_Start
Description:  motor control start function
Input:        nothing (uses some global variables)
Output:       nothing (modifies some global variables)
Note:         called when start button is pressed
******************************************************************************/
void Motor_Start(void)
{
    Motor_Speed_PIParams.integratorBuf = 0;
    Motor_Speed_PIParams.outputvalue = 0;
    Motor_BCParams.actual_speed = 0;
    Motor_BCParams.avgtimestorage = 0;
    Motor_BCParams.avgcycletime = 0;
    Motor_BCParams.avgctr = (uint16_t) MOTOR_SPEED_CALCFACTOR;
    Motor_BCParams.actual_speed_target = 0;
    Motor_BCParams.speed_pi_enable = 0;
    Motor_StateParams.motor_stop_source = NO_START_CMD;

    /* Initialize duty cycle */
    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, 0);

    Motor_BCParams.speed_reference_target = 0;

    Motor_BEMFParams.curpattern_openmode = 1;

    Motor_BEMFParams.curBEMFpattern = Motor_BEMFParams.curpattern_openmode;

    Motor_HallParams.pattern_commutation = COMMUTATION_ARRAY[Motor_BEMFParams.curpattern_openmode + (Motor_StateParams.direction_offset)];

    /* PWM should be applied only at high side switches */
    Motor_HallParams.patt_enable = (uint8_t)(Motor_HallParams.pattern_commutation & 0x00FF);

    Motor_HallParams.patt_value  = (uint8_t)(Motor_HallParams.pattern_commutation >> 8) & 0x00FF;

    TCC0_PWMPatternSet(Motor_HallParams.patt_enable, Motor_HallParams.patt_value);

    Motor_BEMFParams.nextpattern_openmode = HALL_ARRAY[Motor_BEMFParams.curpattern_openmode+(Motor_StateParams.direction_offset)];

    Motor_BEMFParams.nextBEMFpattern = Motor_BEMFParams.nextpattern_openmode;

    Motor_StateParams.motor_control_state = MC_STATE_OPENLOOP;

    /* Disable analog comparator */
    AC_REGS->AC_COMPCTRL[0] &= ~(AC_COMPCTRL_ENABLE_Msk);

    /* Check Synchronization to ensure that the comparator is disabled */
    while((AC_REGS->AC_SYNCBUSY & AC_SYNCBUSY_COMPCTRL0_Msk) == AC_SYNCBUSY_COMPCTRL0_Msk)
    {
        /* Wait for Synchronization */
    }

    /* Clear the interrupt flags*/
    AC_REGS->AC_INTFLAG = AC_INTFLAG_Msk;
    Motor_StateParams.mc_state_counter = 0;

    TC3_TimerStart();
}


/******************************************************************************
Function:     Motor_Stop
Description:  motor control start function
Input:        nothing (uses some global variables)
Output:       nothing (modifies some global variables)
Note:         called when stop button is pressed
******************************************************************************/
void Motor_Stop(void)
{
    TCC0_PWMPatternSet(0x77, 0x00);
    Motor_BCParams.speed_reference_target = 0;
    Motor_BCParams.speed_reference_rpm = 0;
    Motor_BCParams.actual_speed = 0;
    Motor_BCParams.actual_speed_target = 0;
    Motor_BCParams.speed_pi_enable = 0;
    Motor_Speed_PIParams.integratorBuf = 0;
    TC3_TimerStop();

    if(!Motor_StateParams.motor_stop_source)
    {
        Motor_StateParams.motor_stop_source = STOP_CMD;
    }

    Motor_StateParams.stall_counter = 0;
    Motor_StateParams.mc_state_counter = 0;

    TC0_TimerStop();

    /* Clear all interrupt flags */
    TC0_REGS->COUNT16.TC_INTFLAG = TC_INTFLAG_Msk;

    TC2_TimerStop();
    /* Clear all interrupt flags */
    TC2_REGS->COUNT16.TC_INTFLAG = TC_INTFLAG_Msk;


    /* Disable analog comparator */
    AC_REGS->AC_COMPCTRL[0] &= ~(AC_COMPCTRL_ENABLE_Msk);

    /* Check Synchronization to ensure that the comparator is disabled */
    while((AC_REGS->AC_SYNCBUSY & AC_SYNCBUSY_COMPCTRL0_Msk) == AC_SYNCBUSY_COMPCTRL0_Msk)
    {
        /* Wait for Synchronization */
    }

    /* Clear the interrupt flags*/
    AC_REGS->AC_INTFLAG = AC_INTFLAG_Msk;

    Motor_StateParams.motor_control_state = MC_STATE_STOP;

    Motor_SensorlessParams.openloop_ramp_speed = Motor_SensorlessParams.openloop_start_speed;
    Motor_SensorlessParams.openloop_rotor_angle = 0;
    Motor_SensorlessParams.AlignCounter = OPENLOOP_ALIGN_PERIOD*1000;
}



/******************************************************************************
Function:     OC_FAULT_ISR
Description:  Fault interrupt handler
Input:        nothing (uses some global variables)
Output:       nothing (modifies some global variables)
Note:         called when overcurrent is detected
******************************************************************************/
#ifdef RAM_EXECUTE
void __ramfunc__ OC_FAULT_ISR(uintptr_t context)
#else
void OC_FAULT_ISR(uintptr_t context)
#endif
{
    Motor_StateParams.motor_stop_source = OC_FAULT_STOP;
    Motor_StateParams.state_run = 0;
    LED1_OC_FAULT_Set();
    while(1);
}

/******************************************************************************
Function:     ADC_ISR
Description:  ADC result ready interrupt. This reads shunt current and potentiometer.
Input:        nothing (uses some global variables)
Output:       nothing (modifies some global variables)
Note:         called when shunt current measurement is finished.
******************************************************************************/
#ifdef RAM_EXECUTE
void __ramfunc__ ADC_ISR(ADC_STATUS status, uintptr_t context)
#else
void ADC_ISR(ADC_STATUS status, uintptr_t context)
#endif
{
    X2Cscope_Update();
    /* Read the ADC result value */
    Motor_BCParams.speed_ref_pot = ADC1_ConversionResultGet();
    Motor_BCParams.motor_current = ADC0_ConversionResultGet();

    if(!Motor_StateParams.var_time_10ms)
    {
        Motor_StateParams.var_cnt_10ms++;
    }

    if(Motor_StateParams.var_cnt_10ms == COUNT_10_MS)
    {
        Motor_StateParams.var_time_10ms = 1;
        Motor_StateParams.var_cnt_10ms = 0;
    }
    /* Clear all interrupt flags */
    ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_Msk;

        /*Open loop for BLDC sensor-less*/
#ifndef HALL_MODE
    if(Motor_StateParams.state_run)
    {
        if(Motor_SensorlessParams.AlignCounter==0)
        {
            Motor_StateParams.stall_counter++;
        }

        switch (Motor_StateParams.motor_control_state)
        {
            case MC_STATE_OPENLOOP:
            {
                Motor_SensorlessParams.openloop_rotor_angle += Motor_SensorlessParams.openloop_rotor_angle_delta;

                if(Motor_SensorlessParams.openloop_rotor_angle >> 15)
                {
                    BEMF_Period_Measure();   // for transition to BEMF commutation smoothly.

                    SixStep_OpenLoop_Commutation();

                    Motor_BEMFParams.curstate  = MC_STATE_OPENLOOP;

                    Motor_SensorlessParams.openloop_rotor_angle &= 0x7fff;

                    Motor_StateParams.commutation_time = Motor_StateParams.stall_counter;
                    Motor_StateParams.stall_counter = 0;
                }
            }
            break;

            case MC_STATE_START_BEMF_MEAS:
            {
                Motor_SensorlessParams.openloop_rotor_angle += Motor_SensorlessParams.openloop_rotor_angle_delta;

                if(Motor_SensorlessParams.openloop_rotor_angle >> 15)
                {
                    BEMF_Period_Measure();
                    /* Configure timer period */
                    TC2_Timer16bitPeriodSet(Motor_BEMFParams.Commutation_Delay);

                    SixStep_OpenLoop_Commutation();

                    Motor_BEMFParams.curstate  = MC_STATE_START_BEMF_MEAS;

                    if(Motor_StateParams.direction == 1)
                    {
                        Motor_BEMFParams.curBEMF_C = (Motor_BEMFParams.curBEMFpattern>>2) & 0x01;
                        Motor_BEMFParams.curBEMF_A = (Motor_BEMFParams.curBEMFpattern>>1) & 0x01;
                        Motor_BEMFParams.curBEMF_B = Motor_BEMFParams.curBEMFpattern & 0x01;
                    }
                    else
                    {
                        Motor_BEMFParams.curBEMF_A = (Motor_BEMFParams.curBEMFpattern>>2) & 0x01;
                        Motor_BEMFParams.curBEMF_B = (Motor_BEMFParams.curBEMFpattern>>1) & 0x01;
                        Motor_BEMFParams.curBEMF_C = Motor_BEMFParams.curBEMFpattern & 0x01;
                    }

                    TC0_Timer16bitPeriodSet(BLANK_TIME);

                    /* Start the timer channel 0*/
                    TC0_TimerStart();

                    Motor_SensorlessParams.openloop_rotor_angle &= 0x7fff;

                    Motor_StateParams.motor_control_state = MC_STATE_BEMF_TRANSITION;
                    Motor_StateParams.mc_state_counter = MOTOR_SPEED_CALCFACTOR;

                    Motor_StateParams.commutation_time = Motor_StateParams.stall_counter;
                    Motor_StateParams.stall_counter = 0;
                }
            }
            break;
            case MC_STATE_BEMF_TRANSITION:
            {

                Motor_BEMFParams.curstate  = MC_STATE_BEMF_TRANSITION;

                if(Motor_StateParams.stall_counter > (Motor_StateParams.commutation_time <<3))
                {
                    Motor_StateParams.switch_state ^= 1;
                    Motor_StateParams.state_run = 0;
                    Motor_StateParams.motor_stop_source = STALL_FAULT_STOP;
                    Motor_Stop();
                }
            }
            break;
            case MC_STATE_BEMF_COMMUTATION:
            {

                Motor_BEMFParams.curstate  = MC_STATE_BEMF_COMMUTATION;

                if(Motor_StateParams.stall_counter > (Motor_StateParams.commutation_time <<3))
                {
                    Motor_StateParams.switch_state ^= 1;
                    Motor_StateParams.state_run = 0;
                    Motor_StateParams.motor_stop_source = STALL_FAULT_STOP;
                    Motor_Stop();
                }
            }
            break;
            case MC_STATE_SPEEDLOOP_INITIAL:
            {

                Motor_BEMFParams.curstate  = MC_STATE_SPEEDLOOP_INITIAL;

                if(Motor_StateParams.stall_counter > (Motor_StateParams.commutation_time <<3))
                {
                    Motor_StateParams.switch_state ^= 1;
                    Motor_StateParams.state_run = 0;
                    Motor_StateParams.motor_stop_source = STALL_FAULT_STOP;
                    Motor_Stop();
                }
            }
            break;
            case MC_STATE_SPEEDLOOP_CLOSED:
            {

                Motor_BEMFParams.curstate  = MC_STATE_SPEEDLOOP_CLOSED;

                if(Motor_StateParams.stall_counter > (Motor_StateParams.commutation_time <<3))
                {
                    Motor_StateParams.switch_state ^= 1;
                    Motor_StateParams.state_run = 0;
                    Motor_StateParams.motor_stop_source = STALL_FAULT_STOP;
                    Motor_Stop();
                }
            }
            break;
            default:

            break;
        }
    }
#endif
    return;
}

/******************************************************************************
Function:     Hall_UpdateCommutation_ISR
Description:  Hall pin edge detect interrupt.
 Next commutation pattern is updated in buffer register. And speed is calculated based on
 time captured by the timer TC3.
Input:        nothing (uses some global variables)
Output:       nothing (modifies some global variables)
Note:         called when edge is detected on any hall pin
******************************************************************************/
#ifdef RAM_EXECUTE
void __ramfunc__ Hall_UpdateCommutation_ISR(void)
#else
void Hall_UpdateCommutation_ISR(void)
#endif
{

}

/******************************************************************************
Function:     TC4_1ms_ISR
Description:  1 mS interrupt handler  Speed Control loop
Input:        nothing (uses some global variables)
Output:       nothing (modifies some global variables)
Note:         called every 1 mS
******************************************************************************/
uint16_t debug_duty;
#ifdef RAM_EXECUTE
void __ramfunc__ TC4_1ms_ISR(TC_TIMER_STATUS status, uintptr_t context)
#else
void TC4_1ms_ISR(TC_TIMER_STATUS status, uintptr_t context)
#endif
{
    uint16_t duty_pwm;

    Motor_BCParams.set_speed_target = (Motor_BCParams.speed_ref_pot << Q_NUM) / MAX_POT_REF;

    if (Motor_BCParams.set_speed_target<Motor_SensorlessParams.openloop_end_speed)
    {
        Motor_BCParams.set_speed_target = (Motor_SensorlessParams.openloop_end_speed);
    }
    else ;

    /*Open loop for BLDC sensor-less*/
    if(Motor_StateParams.state_run)
    {
        switch (Motor_StateParams.motor_control_state)
        {
            case MC_STATE_OPENLOOP:
            {
                if(Motor_SensorlessParams.AlignCounter==0)
                {
                    Motor_BCParams.speed_reference_target = speed_ramp(Motor_SensorlessParams.openloop_end_speed,MOTOR_RAMPUP_SPEED_PER_MS,Motor_BCParams.speed_reference_target);
                    Motor_BCParams.speed_reference_rpm = (Motor_BCParams.speed_reference_target * MAX_MOTOR_SPEED) >> Q_NUM;

                    Motor_SensorlessParams.openloop_rotor_angle_delta = (Motor_BCParams.speed_reference_rpm * Motor_SensorlessParams.RPMtoAngle_factor)>>Q_NUM;

                    if((Motor_BCParams.speed_reference_target == Motor_SensorlessParams.openloop_end_speed))
                    {
#ifndef OPENLOOP_MODE
                        Motor_StateParams.motor_control_state =  MC_STATE_START_BEMF_MEAS;
#endif
                    }
                    else
                        ;
                }
                else
                {
                    if(Motor_SensorlessParams.Enbale_Align == 1)
                    {
                        Motor_SensorlessParams.AlignCounter--;
                    }
                    else
                    {
                        Motor_SensorlessParams.AlignCounter = 0;
                    }
                    Motor_SensorlessParams.openloop_rotor_angle_delta = 0;
                }

                duty_pwm = ((Motor_BCParams.speed_reference_rpm * Motor_SensorlessParams.openloop_pwm_factor) >>Q_NUM)  + Motor_SensorlessParams.openloop_pwm_offset;
                debug_duty = duty_pwm;
                TCC0_PWM24bitDutySet(TCC0_CHANNEL0, duty_pwm);
            }
            break;

            case MC_STATE_BEMF_COMMUTATION:
            {
                ;
            }
            break;
            case MC_STATE_SPEEDLOOP_CLOSED:
            {
                    Motor_BCParams.speed_reference_target = speed_ramp(Motor_BCParams.set_speed_target,MOTOR_RAMPUP_SPEED_PER_MS,Motor_BCParams.speed_reference_target);
                    Motor_BCParams.speed_reference_rpm = (Motor_BCParams.speed_reference_target * MAX_MOTOR_SPEED) >> Q_NUM;
                    /*    Error Calculation   Error = Reference - Actual    */
                    Motor_Speed_PIParams.error    = (int32_t)Motor_BCParams.speed_reference_target - (int32_t)Motor_BCParams.actual_speed_target;
                    duty_pwm = pi_lib_calculate(&Motor_Speed_PIParams);
                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, duty_pwm);
            }
            break;
            case MC_STATE_STOP:
            {

            }
            break;
            default:

            break;
        }
    }

}

uint16_t bemf_pattern;
/*BEMF zero-crossing ISR*/
#ifdef RAM_EXECUTE
void __ramfunc__ ac_callBack(uint8_t int_flag, uintptr_t ac_context)
#else
void ac_callBack(uint8_t int_flag, uintptr_t ac_context)
#endif
{

    if(Motor_StateParams.state_run)
    {
        if(Motor_BEMFParams.undriven_phase == phase_U)
        {

            Motor_BEMFParams.curBEMF_A = AC_StatusGet(AC_CHANNEL0);

        }
        else if(Motor_BEMFParams.undriven_phase == phase_V)
        {

            Motor_BEMFParams.curBEMF_B = AC_StatusGet(AC_CHANNEL0);

        }
        else if(Motor_BEMFParams.undriven_phase == phase_W)
        {

            Motor_BEMFParams.curBEMF_C = AC_StatusGet(AC_CHANNEL0);

        }
        else
            ;

        if(Motor_StateParams.direction ==1 )
        {
            Motor_BEMFParams.curBEMFpattern = ((Motor_BEMFParams.curBEMF_C << 2)|(Motor_BEMFParams.curBEMF_A << 1)| Motor_BEMFParams.curBEMF_B);
        }
        else
        {
           Motor_BEMFParams.curBEMFpattern = ((Motor_BEMFParams.curBEMF_A << 2)|(Motor_BEMFParams.curBEMF_B << 1)| Motor_BEMFParams.curBEMF_C);
        }

        bemf_pattern = Motor_BEMFParams.curBEMFpattern;

#ifndef HALL_MODE
        switch (Motor_StateParams.motor_control_state)
        {

            case MC_STATE_BEMF_TRANSITION:
            {
                BEMF_Period_Measure();

                /* Start the timer channel*/
                TC2_TimerStart();

                Motor_StateParams.mc_state_counter--;

                if(Motor_StateParams.mc_state_counter==0)
                {
                    Motor_StateParams.motor_control_state = MC_STATE_BEMF_COMMUTATION;
                    Motor_StateParams.mc_state_counter = MOTOR_BEMF_COMMUTATION_WAIT_CNT;
                }
            }
            break;

            case MC_STATE_BEMF_COMMUTATION:
            {
                BEMF_Period_Measure();

                /* Configure timer period */
                TC2_Timer16bitPeriodSet(Motor_BEMFParams.Commutation_Delay);

                /* Start the timer channel*/
                TC2_TimerStart();

                Motor_StateParams.mc_state_counter--;

                if(Motor_StateParams.mc_state_counter==0)
                {
                    Motor_StateParams.motor_control_state = MC_STATE_SPEEDLOOP_INITIAL;
                }
            }
            break;

            case MC_STATE_SPEEDLOOP_INITIAL:
            {
                BEMF_Period_Measure();

                /* Configure timer period */
                TC2_Timer16bitPeriodSet(Motor_BEMFParams.Commutation_Delay);

                /* Start the timer channel*/
                TC2_TimerStart();

                Motor_Speed_PIParams.integratorBuf = (uint32_t)(Motor_SensorlessParams.openloop_end_pwm + Motor_SensorlessParams.openloop_pwm_offset)<<15;
                Motor_BCParams.speed_pi_enable = 1;

                Motor_StateParams.motor_control_state = MC_STATE_SPEEDLOOP_CLOSED;
            }
            break;

            case MC_STATE_SPEEDLOOP_CLOSED:
            {
                BEMF_Period_Measure();

                /* Configure timer period */
                TC2_Timer16bitPeriodSet(Motor_BEMFParams.Commutation_Delay);

                /* Start the timer channel*/
                TC2_TimerStart();
            }
            break;

            default:
            break;
        }
        if(Motor_BCParams.speed_pi_enable)
        {
            if (Motor_BCParams.avgcycletime != 0)
            {
                Motor_BCParams.actual_speed = Motor_BCParams.speed_constant / (Motor_BCParams.avgcycletime);
            }
        }
        Motor_BCParams.actual_speed_target = (Motor_BCParams.actual_speed << 14) / MAX_MOTOR_SPEED;
#endif
    }

}


void SixStep_OpenLoop_Commutation(void)
{
    Motor_BEMFParams.curBEMFpattern = Motor_BEMFParams.nextBEMFpattern;

    Motor_HallParams.pattern_commutation = COMMUTATION_ARRAY[Motor_BEMFParams.curBEMFpattern +
                                                                    (Motor_StateParams.direction_offset)];

    Motor_BEMFParams.nextBEMFpattern = HALL_ARRAY[Motor_BEMFParams.curBEMFpattern + (Motor_StateParams.direction_offset)];


    Motor_HallParams.patt_enable = (uint8_t) (Motor_HallParams.pattern_commutation & 0x00FF);
    Motor_HallParams.patt_value  = (uint8_t) (Motor_HallParams.pattern_commutation >> 8) & 0x00FF;

    /* Set PWM pattern */
        TCC0_PWMPatternSet(Motor_HallParams.patt_enable, Motor_HallParams.patt_value);

     Read_UndrivenPhase();
     
    AC_REGS->AC_COMPCTRL[0] |= AC_COMPCTRL_ENABLE_Msk;
    while((AC_REGS->AC_SYNCBUSY & AC_SYNCBUSY_COMPCTRL0_Msk) == AC_SYNCBUSY_COMPCTRL0_Msk)
    {
        /* Wait for Synchronization */
    }
}

/*TC2 interrupt for commutation*/
#ifdef RAM_EXECUTE
void __ramfunc__ BEMF_Commutation(TC_TIMER_STATUS status, uintptr_t context)
#else
void BEMF_Commutation(TC_TIMER_STATUS status, uintptr_t context)
#endif
{
    if ( (Motor_BEMFParams.curBEMFpattern == Motor_BEMFParams.nextBEMFpattern) &&
            (Motor_BEMFParams.curBEMFpattern != 0 && Motor_BEMFParams.curBEMFpattern != 7))
    {
        Motor_StateParams.commutation_time = Motor_StateParams.stall_counter;
        Motor_StateParams.stall_counter = 0;

        Motor_StateParams.wrong_commutation_counter = 0;


        Motor_HallParams.pattern_commutation = COMMUTATION_ARRAY[Motor_BEMFParams.curBEMFpattern +
                                                                (Motor_StateParams.direction_offset)];

        Motor_HallParams.patt_enable = (uint8_t) (Motor_HallParams.pattern_commutation & 0x00FF);
        Motor_HallParams.patt_value  = (uint8_t) (Motor_HallParams.pattern_commutation >> 8) & 0x00FF;

        /* Set PWM pattern */
        TCC0_PWMPatternSet(Motor_HallParams.patt_enable, Motor_HallParams.patt_value);

        Motor_BEMFParams.nextBEMFpattern = HALL_ARRAY[Motor_BEMFParams.curBEMFpattern + (Motor_StateParams.direction_offset)];

        Read_UndrivenPhase();

        /* Start TC0 timer to enable the analog comparator after blank time in  'AC_blank_end' */
        TC0_Timer16bitPeriodSet(BLANK_TIME);

        /* Start the timer channel 0*/
        TC0_TimerStart();
    }
    else 
    {
        Motor_StateParams.wrong_commutation_counter++;
        if(Motor_StateParams.wrong_commutation_counter > 2)
        {
            Motor_StateParams.switch_state ^= 1;
            Motor_StateParams.state_run = 0;
            Motor_StateParams.motor_stop_source = COMMUTATION_FAULT_STOP;
            Motor_Stop();
        }
    }

}

/*TC0 interrupt Blank Time Ending */
void AC_blank_end(TC_TIMER_STATUS status, uintptr_t context)
{
        /* Clear the interrupt flags*/
    AC_REGS->AC_INTFLAG = AC_INTFLAG_Msk;
    
    AC_REGS->AC_COMPCTRL[0] |= AC_COMPCTRL_ENABLE_Msk;

    while((AC_REGS->AC_SYNCBUSY & AC_SYNCBUSY_COMPCTRL0_Msk) == AC_SYNCBUSY_COMPCTRL0_Msk)
    {
        /* Wait for Synchronization */
    }
}


uint16_t speed_ramp(uint16_t Ref, uint16_t ramp_rate, uint16_t ref_temp)
{

    if (ramp_rate == 0U)
    {
        ref_temp = Ref;
    }
    else
    {
        if(Ref > (ref_temp + ramp_rate) )
        {
            ref_temp += ramp_rate;
        }
        else if(Ref < (ref_temp - ramp_rate))
        {
            ref_temp -= ramp_rate;
        }
        else
        {
            ref_temp = Ref;
        }
    }

    return ref_temp;
}

void BEMF_Period_Measure(void)
{
    uint16_t timeelapsed;

    /* Read the time elapsed from last hall pattern */
    timeelapsed = TC3_Timer16bitCounterGet();
    TC3_TimerStop();

    TC3_TimerStart();

    if (Motor_BCParams.avgctr == 0)
    {
        Motor_BCParams.avgcycletime = Motor_BCParams.avgtimestorage / MOTOR_SPEED_CALCFACTOR;
        Motor_BCParams.avgtimestorage -= Motor_BCParams.avgcycletime;

    }
    else
    {
        Motor_BCParams.avgctr--;

    }
    Motor_BCParams.avgtimestorage += timeelapsed;


    if((Motor_BCParams.avgcycletime) > COMMUTATION_MAX_DELAY)
    {
        Motor_BEMFParams.Commutation_Delay = COMMUTATION_MAX_DELAY>>1;
    }
    else if(Motor_BCParams.avgcycletime <= (COMMUTE_COMPENSATION<<1))
    {
        Motor_BEMFParams.Commutation_Delay = COMMUTATION_MIN_DELAY;
    }
    else
    {
        Motor_BEMFParams.Commutation_Delay = (Motor_BCParams.avgcycletime >> 1) - COMMUTE_COMPENSATION;
    }

}



void Read_UndrivenPhase(void)
{
    AC_REGS->AC_COMPCTRL[0] &= ~AC_COMPCTRL_ENABLE_Msk;
    while((AC_REGS->AC_SYNCBUSY & AC_SYNCBUSY_COMPCTRL0_Msk) == AC_SYNCBUSY_COMPCTRL0_Msk)
    {
        /* Wait for Synchronization */
    }


        if((Motor_HallParams.pattern_commutation == 0x4075 )||(Motor_HallParams.pattern_commutation == 0x2073 ))  // Un-driven Phase U;
        {
            Motor_BEMFParams.undriven_phase = phase_U;
            
            AC_REGS->AC_COMPCTRL[0] &= ~(AC_COMPCTRL_MUXPOS_Msk | AC_COMPCTRL_MUXNEG_Msk);
            AC_REGS->AC_COMPCTRL[0] |= ((uint32_t)AC_COMPCTRL_MUXPOS_AIN3 | (uint32_t)AC_COMPCTRL_MUXNEG_AIN0 );
        }
         else if((Motor_HallParams.pattern_commutation == 0x4076 )||(Motor_HallParams.pattern_commutation == 0x1073 )) // Un-driven Phase V;
        {
            Motor_BEMFParams.undriven_phase = phase_V;

            AC_REGS->AC_COMPCTRL[0] &= ~(AC_COMPCTRL_MUXPOS_Msk | AC_COMPCTRL_MUXNEG_Msk);
            AC_REGS->AC_COMPCTRL[0] |= ((uint32_t)AC_COMPCTRL_MUXPOS_AIN3 | (uint32_t)AC_COMPCTRL_MUXNEG_AIN1 );
        }
        else if((Motor_HallParams.pattern_commutation == 0x2076 )||(Motor_HallParams.pattern_commutation == 0x1075 )) // Un-driven Phase W;
        {
            Motor_BEMFParams.undriven_phase = phase_W;

            AC_REGS->AC_COMPCTRL[0] &= ~(AC_COMPCTRL_MUXPOS_Msk | AC_COMPCTRL_MUXNEG_Msk);
            AC_REGS->AC_COMPCTRL[0] |= ((uint32_t)AC_COMPCTRL_MUXPOS_AIN3 | (uint32_t)AC_COMPCTRL_MUXNEG_AIN2 );
        }
        else
        {
            Motor_BEMFParams.undriven_phase = 3;
        }
}
 
/* End of mc_app.c */


