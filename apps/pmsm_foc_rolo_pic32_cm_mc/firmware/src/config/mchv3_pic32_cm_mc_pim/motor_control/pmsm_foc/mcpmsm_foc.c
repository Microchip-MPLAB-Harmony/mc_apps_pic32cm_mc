/*******************************************************************************
 Motor Control Application Source file 

  Company:
    Microchip Technology Inc.

  File Name:
    mc_pmsm_foc.c

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
#include <stdlib.h>
#include "definitions.h"
#include <sys/attribs.h>
#include "mc_motor_control.h"
#include "mc_start_up.h"

/*******************************************************************************
Interface Functions
*******************************************************************************/
/*! \brief Motor Control functions initialization 
 *  
 *  Details.
 *  Initializes software components for PMSM motor control function 
 
 */
void PMSM_FOC_Initialize( void )
{
     /* Initialize application */
     mcFco_ApplicationInitialize( );
}


/*! \brief Main loop tasks 
 *  
 *  Details.
 *  Main Loop tasks  
 
 */
uint8_t test = 0;
void PMSM_FOC_Tasks( void )
{
     /* Initialize application */
     mcFco_TaskManagement( );
}

/*! \brief Start motor
 *  
 *  Details.
 *  Interface to start the motor
 
 */
void PMSM_FOC_MotorStart( void )
{
    mcMocI_MotorStart();
}

/*! \brief Motor stop
 *  
 *  Details.
 *  Interface function to stop the motor.
 
 */
void PMSM_FOC_MotorStop( void )
{
    mcMocI_MotorStop();
}

/*! \brief Motor torque set
 *  
 *  Details.
 *  Interface function to set the motor torque.
 
 */
void PMSM_FOC_TorqueSet( float f_torque_df32 )
{

}

/*! \brief Set motor speed 
 *  
 *  Details.
 *  Interface function to set the motor speed. 
 
 */
void PMSM_FOC_MotorRpmSet( const int16_t motorSpeed )
{
    
}

/*! \brief Get motor speed 
 *  
 *  Details.
 *  Interface function to get the motor speed. 
 
 */
int16_t PMSM_FOC_RotorSpeedGet( void )
{
    return 0;
}

/*! \brief Get rotor position  
 *  
 *  Details.
 *  Interface function to get the motor position. 
 
 */
float PMSM_FOC_RotorPositionGet( void )
{
    return 0;
}

/*! \brief 1 ms task callback function
 *  
 *  Details.
 *  1 ms task callback function. 
 
 */
void PMSM_FOC_1msCallbackRegister( void )
{
    
}

/*! \brief 10 ms task callback function
 *  
 *  Details.
 *  10 ms task callback function
 
 */
void PMSM_FOC_10msCallbackRegister( void )
{
    
}

/*! \brief 100 ms task callback function
 *  
 *  Details.
 *  100 ms task callback function
 
 */
void PMSM_FOC_100msCallbackRegister( void )
{
    
}

/*! \brief 1000 ms task callback function
 *  
 *  Details.
 *  1000 ms task callback function
 
 */
void PMSM_FOC_1000msCallbackRegister( void )
{
    
}

/* EOF motor_control.c */

