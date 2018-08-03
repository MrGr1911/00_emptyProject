/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include "driverlib.h"
#include "Initial.h"



#define TIMER_PERIOD 50
#define DUTY_CYCLE1 32
#define DUTY_CYCLE2 25
//******************************************************************************
//!
//!   Empty Project that includes driverlib
//!
//******************************************************************************
void Initial ( void );
void GpioInitial ( void );
void LeftTurn ( void );
void RightTurn ( void );
void Stop ( void );
void Stright ( void );
void pwmControl ( unsigned int pwmL, unsigned int pwmR );
void PwmInitial ( void );

void main ( void )
{
    unsigned int Left, Right, pwmL, pwmR;
    //Stop WDT
    WDT_A_hold ( WDT_A_BASE );
    Initial();
    pwmControl ( 10, 10 );

    while ( 1 )
    {
        Left = GPIO_getInputPinValue ( GPIO_PORT_P3, GPIO_PIN7 );
        Right = GPIO_getInputPinValue ( GPIO_PORT_P8, GPIO_PIN2 );

        if ( Left && Right )
            Stop();
        else if ( Left )
            RightTurn();
        else if ( Right )
            LeftTurn();
        else
            Stright();
    }
}

void Initial ( void )
{
    GpioInitial();
    PwmInitial();
}

void GpioInitial ( void )
{
    GPIO_setAsOutputPin ( GPIO_PORT_P4, GPIO_PIN0 + GPIO_PIN3 );
    GPIO_setAsOutputPin ( GPIO_PORT_P1, GPIO_PIN2 + GPIO_PIN3 );
    GPIO_setAsInputPin ( GPIO_PORT_P3, GPIO_PIN7 );
    GPIO_setAsInputPin ( GPIO_PORT_P8, GPIO_PIN2 );
    //pwm pin Initial
    GPIO_setAsPeripheralModuleFunctionOutputPin ( GPIO_PORT_P1, GPIO_PIN4 + GPIO_PIN5 );
    GPIO_setDriveStrength ( GPIO_PORT_P1, GPIO_PIN4 + GPIO_PIN5, GPIO_FULL_OUTPUT_DRIVE_STRENGTH );
}

void LeftTurn()
{
    GPIO_setOutputHighOnPin ( GPIO_PORT_P4, GPIO_PIN3 );
    GPIO_setOutputHighOnPin ( GPIO_PORT_P1, GPIO_PIN3 );
    GPIO_setOutputLowOnPin ( GPIO_PORT_P4, GPIO_PIN0 );
    GPIO_setOutputLowOnPin ( GPIO_PORT_P1, GPIO_PIN2 );
}


void RightTurn()
{
    GPIO_setOutputHighOnPin ( GPIO_PORT_P4, GPIO_PIN0 );
    GPIO_setOutputHighOnPin ( GPIO_PORT_P1, GPIO_PIN2 );
    GPIO_setOutputLowOnPin ( GPIO_PORT_P4, GPIO_PIN3 );
    GPIO_setOutputLowOnPin ( GPIO_PORT_P1, GPIO_PIN3 );
}


void Stop()
{
    GPIO_setOutputHighOnPin ( GPIO_PORT_P4, GPIO_PIN0 );
    GPIO_setOutputHighOnPin ( GPIO_PORT_P4, GPIO_PIN3 );
    GPIO_setOutputHighOnPin ( GPIO_PORT_P1, GPIO_PIN2 );
    GPIO_setOutputHighOnPin ( GPIO_PORT_P1, GPIO_PIN3 );
}


void Stright()
{
    GPIO_setOutputHighOnPin ( GPIO_PORT_P4, GPIO_PIN0 );
    GPIO_setOutputHighOnPin ( GPIO_PORT_P1, GPIO_PIN3 );
    GPIO_setOutputLowOnPin ( GPIO_PORT_P4, GPIO_PIN3 );
    GPIO_setOutputLowOnPin ( GPIO_PORT_P1, GPIO_PIN2 );
}

void pwmControl ( unsigned int pwmL, unsigned int pwmR )
{
    Timer_A_setCompareValue ( TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, pwmL );
    Timer_A_setCompareValue ( TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, pwmR );
}

void PwmInitial()
{
    //Start Timer
    Timer_A_initUpDownModeParam initUpDownParam = {0};
    initUpDownParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initUpDownParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initUpDownParam.timerPeriod = TIMER_PERIOD;
    initUpDownParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initUpDownParam.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    initUpDownParam.timerClear = TIMER_A_DO_CLEAR;
    initUpDownParam.startTimer = false;
    Timer_A_initUpDownMode ( TIMER_A0_BASE, &initUpDownParam );
    Timer_A_startCounter ( TIMER_A0_BASE, TIMER_A_UPDOWN_MODE );
    //Initialze compare registers to generate PWM1
    Timer_A_initCompareModeParam initComp3Param = {0};
    initComp3Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    initComp3Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    initComp3Param.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;
    initComp3Param.compareValue = DUTY_CYCLE1;
    Timer_A_initCompareMode ( TIMER_A0_BASE, &initComp3Param );
    //Initialze compare registers to generate PWM2
    Timer_A_initCompareModeParam initComp4Param = {0};
    initComp4Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    initComp4Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    initComp4Param.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;
    initComp4Param.compareValue = DUTY_CYCLE2;
    Timer_A_initCompareMode ( TIMER_A0_BASE, &initComp4Param );
}