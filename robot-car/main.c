/*
 * Copyright (c) 2012, Mauro Scomparin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Mauro Scomparin nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Mauro Scomparin ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Mauro Scomparin BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File:			main.c.
 * Author:		Mauro Scomparin <http://scompoprojects.worpress.com>.
 * Version:		1.0.0.
 * Description:	Main sample file.
 */

#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"

#include "sonar-belt.h"
#include "servos.h"


#define LED_PORT GPIO_PORTF_BASE
#define LED_R	GPIO_PIN_1
#define LED_B	GPIO_PIN_2
#define LED_G	GPIO_PIN_3

#define SW_PORT GPIO_PORTF_BASE
#define SW1		GPIO_PIN_4
#define SW2		GPIO_PIN_0

unsigned char sonarFirstEdge = 0, pulseGen = 0;
unsigned long int sonarTemp = 0;


struct sonar_belt ranger_belt;
struct servo_array servos;

// An interrupt function.
void Timer1A_ISR(void);
void Timer1B_ISR(void);
void GPIO_Sonar_ISR(void);

// main function.
int main(void) {
	unsigned int i = 0, k = 0, j = 0;
	float rangeMin = 0.0;
	float angle_sweep = -90.0;
	unsigned char emit = 1;
	FPULazyStackingEnable();
	FPUEnable();
	SysCtlClockSet(
			SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
					| SYSCTL_XTAL_16MHZ); //80Mhz

	initBelt(&ranger_belt, 6);

	ranger_belt.ranger[0].trigPort = SONAR_1_TRIG_PORT;
	ranger_belt.ranger[0].echoPort = SONAR_1_ECHO_PORT;
	ranger_belt.ranger[0].echoPin = SONAR_1_ECHO;
	ranger_belt.ranger[0].trigPin = SONAR_1_TRIG;
	ranger_belt.ranger[0].heading = -75.0;
	ranger_belt.ranger[0].cone = 30.0;

	ranger_belt.ranger[1].trigPort = SONAR_2_TRIG_PORT;
	ranger_belt.ranger[1].echoPort = SONAR_2_ECHO_PORT;
	ranger_belt.ranger[1].echoPin = SONAR_2_ECHO;
	ranger_belt.ranger[1].trigPin = SONAR_2_TRIG;
	ranger_belt.ranger[1].heading = -45.0;
	ranger_belt.ranger[1].cone = 30.0;

	ranger_belt.ranger[2].trigPort = SONAR_3_TRIG_PORT;
	ranger_belt.ranger[2].echoPort = SONAR_3_ECHO_PORT;
	ranger_belt.ranger[2].echoPin = SONAR_3_ECHO;
	ranger_belt.ranger[2].trigPin = SONAR_3_TRIG;
	ranger_belt.ranger[2].heading = -15.0;
	ranger_belt.ranger[2].cone = 30.0;

	ranger_belt.ranger[3].trigPort = SONAR_4_TRIG_PORT;
	ranger_belt.ranger[3].echoPort = SONAR_4_ECHO_PORT;
	ranger_belt.ranger[3].echoPin = SONAR_4_ECHO;
	ranger_belt.ranger[3].trigPin = SONAR_4_TRIG;
	ranger_belt.ranger[3].heading = 15.0;
	ranger_belt.ranger[3].cone = 30.0;

	ranger_belt.ranger[4].trigPort = SONAR_5_TRIG_PORT;
	ranger_belt.ranger[4].echoPort = SONAR_5_ECHO_PORT;
	ranger_belt.ranger[4].echoPin = SONAR_5_ECHO;
	ranger_belt.ranger[4].trigPin = SONAR_5_TRIG;
	ranger_belt.ranger[4].heading = 45.0;
	ranger_belt.ranger[4].cone = 30.0;

	ranger_belt.ranger[5].trigPort = SONAR_6_TRIG_PORT;
	ranger_belt.ranger[5].echoPort = SONAR_6_ECHO_PORT;
	ranger_belt.ranger[5].echoPin = SONAR_6_ECHO;
	ranger_belt.ranger[5].trigPin = SONAR_6_TRIG;
	ranger_belt.ranger[5].heading = 75.0;
	ranger_belt.ranger[5].cone = 30.0;

	initServos(&servos, 2);
	servos.servos[0].port = SERVO_1_PORT;
	servos.servos[1].port = SERVO_2_PORT;
	servos.servos[0].pin = SERVO_1_PIN;
	servos.servos[1].pin = SERVO_2_PIN;

	//GPIO_CONFIG: must config each sonar ranger port as gpio interrupt on BOTH_EDGE
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // sonar connected ports
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	for (i = 0; i < ranger_belt.nb; i++) {
		GPIOPinTypeGPIOOutput(ranger_belt.ranger[i].trigPort,
				ranger_belt.ranger[i].trigPin);
		GPIOPinWrite(ranger_belt.ranger[i].trigPort,
				ranger_belt.ranger[i].trigPin, 0); //clear all sonar pins
		GPIOPinTypeGPIOInput(ranger_belt.ranger[i].echoPort,
				ranger_belt.ranger[i].echoPin);
		GPIOIntTypeSet(ranger_belt.ranger[i].echoPort,
				ranger_belt.ranger[i].echoPin, GPIO_BOTH_EDGES);
		GPIOPortIntRegister(ranger_belt.ranger[i].echoPort, GPIO_Sonar_ISR);
	}

	//configuring servo pins as out
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	for (i = 0; i < servos.nb; i++) {
		GPIOPinTypeGPIOOutput(servos.servos[i].port, servos.servos[i].pin);
		GPIOPinWrite(servos.servos[i].port, servos.servos[i].pin, 0); //clear all servo pins*/

	}

	GPIOPinTypeGPIOOutput(LED_PORT, LED_G);
	GPIOPinTypeGPIOOutput(LED_PORT, LED_B);
	HWREG(SW_PORT + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(SW_PORT + GPIO_O_CR) = 0x01;
	HWREG(SW_PORT + GPIO_O_LOCK) = 0;
	GPIOPadConfigSet(SW_PORT, SW2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPinTypeGPIOInput(SW_PORT, SW2);

	//TIMER1_A_CONFIG : used for sonar pulses
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE,
			TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT); //one shot timer, half used
	TimerPrescaleSet(TIMER1_BASE, TIMER_A, 80); // set time prescaler to generate 1us count
	TimerControlStall(TIMER1_BASE, TIMER_A, true);
	TimerLoadSet(TIMER1_BASE, TIMER_A, 15); //first pulse generation
	TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1A_ISR);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//TIME1_B_CONFIG : used for servo pulses
	//TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_ONE_SHOT); //one shot timer, half used
	TimerPrescaleSet(TIMER1_BASE, TIMER_B, 80); // set time prescaler to generate 1us count
	TimerControlStall(TIMER1_BASE, TIMER_B, true);
	TimerLoadSet(TIMER1_BASE, TIMER_B, 20000UL); //first pulse generation
	TimerIntRegister(TIMER1_BASE, TIMER_B, Timer1B_ISR);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

	while (GPIOPinRead(ranger_belt.ranger[0].echoPort,
			ranger_belt.ranger[0].echoPin))
		;
	//generate first pulse to initiate measuring cycle
	pulseGen = 0;
	sonarFirstEdge = 1;
	GPIOPinWrite(ranger_belt.ranger[0].trigPort, ranger_belt.ranger[0].trigPin,
			ranger_belt.ranger[0].trigPin); //set trig
	TimerEnable(TIMER1_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_B);

	//
	// Initialize the UART
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//UARTStdioInitExpClk(0, 9600);  // Baud=9600,
	UARTStdioInit(0); //Baud = 115200

	while (GPIOPinRead(SW_PORT, SW2))
		;
	while (1) {
		k = -1;
		rangeMin = 200.0;
		for (i = 0; i < ranger_belt.nb; i++) {
			if (ranger_belt.ranger[i].lastDistance < 20.0
					&& ranger_belt.ranger[i].lastDistance < rangeMin) {
				k = i;
			}
		}
		if (k > 0) {
			//really arguable obstacle avoidance method
			//should rely on heading order
			if(ranger_belt.ranger[k].heading < 0){
				setServoAngle(&servos.servos[0], ranger_belt.ranger[k].heading + 90);
			}else{
				setServoAngle(&servos.servos[0], ranger_belt.ranger[k].heading - 90);
			}
			GPIOPinWrite(LED_PORT, LED_G, 0);
		} else {
			GPIOPinWrite(LED_PORT, LED_G, LED_G);
			setServoAngle(&servos.servos[0], 0.0);
		}

		if (ranger_belt.curent_measure != 0) {
			emit = 1;
		} else if (ranger_belt.curent_measure == 0 && emit == 1) { // on falling edge of pulseGen
			setServoAngle(&servos.servos[1], angle_sweep);
			angle_sweep++;
			if (angle_sweep > 90.0) {
				angle_sweep = -90.0;
			}
			UARTprintf("[ ");
			for (i = 0; i < ranger_belt.nb; i++) {
				UARTprintf("%d",
						((unsigned int) ranger_belt.ranger[i].lastDistance));
				if (i != (ranger_belt.nb - 1)) {
					UARTprintf(", ");
				} else {
					UARTprintf(" ");
				}
			}
			UARTprintf("]\r\n");
			emit = 0;
		}
		k = pulseGen;
	}
}

//the GPIO interrupt handler
void GPIO_Sonar_ISR(void) {
	GPIOPinIntClear(ranger_belt.ranger[ranger_belt.curent_measure].echoPort,
			ranger_belt.ranger[ranger_belt.curent_measure].echoPin);
	//should be generated first on rising edge then falling edge
	unsigned long int valPins = GPIOPinRead(
			ranger_belt.ranger[ranger_belt.curent_measure].echoPort,
			ranger_belt.ranger[ranger_belt.curent_measure].echoPin); // check pin state
	if (sonarFirstEdge && valPins) { //rising edge on pin
		sonarTemp = TimerValueGet(TIMER1_BASE, TIMER_A);
		sonarFirstEdge = 0;
	} else if (!valPins && !sonarFirstEdge) { // falling edge
		unsigned short int timerVal = TimerValueGet(TIMER1_BASE, TIMER_A);
		timerVal = sonarTemp - timerVal;
		ranger_belt.ranger[ranger_belt.curent_measure].lastDistance =
				(((float) timerVal) / 58.0);
		GPIOPinIntDisable(
				ranger_belt.ranger[ranger_belt.curent_measure].echoPort,
				ranger_belt.ranger[ranger_belt.curent_measure].echoPin);
	}
}

// The timer A interrupt function definition.
void Timer1A_ISR(void) {
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	if (pulseGen) {
		//1Oµs pulse
		ranger_belt.curent_measure = (ranger_belt.curent_measure + 1)
				% ranger_belt.nb;
		struct sonar_ranger currRanger =
				ranger_belt.ranger[ranger_belt.curent_measure];
		TimerLoadSet(TIMER1_BASE, TIMER_A, 15);
		GPIOPinIntDisable(
				ranger_belt.ranger[ranger_belt.curent_measure].echoPort,
				ranger_belt.ranger[ranger_belt.curent_measure].echoPin);
		GPIOPinWrite(ranger_belt.ranger[ranger_belt.curent_measure].trigPort,
				currRanger.trigPin, currRanger.trigPin); //set trig
		pulseGen = 0;
	} else {
		GPIOPinWrite(ranger_belt.ranger[ranger_belt.curent_measure].trigPort,
				ranger_belt.ranger[ranger_belt.curent_measure].trigPin, 0); //clear trig
		GPIOPinIntEnable(
				ranger_belt.ranger[ranger_belt.curent_measure].echoPort,
				ranger_belt.ranger[ranger_belt.curent_measure].echoPin);
		TimerLoadSet(TIMER1_BASE, TIMER_A, 35000UL); // timer resolution is 1µs, counting 64ms
		sonarFirstEdge = 1;
		pulseGen = 1;
	}
	TimerEnable(TIMER1_BASE, TIMER_A);
}

void Timer1B_ISR(void) {
	TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
	GPIOPinWrite(servos.servos[servos.curr].port,
			servos.servos[servos.curr].pin, 0);
	if (servos.remains != 20000UL) { //not a initial count ...
		servos.curr++;
	}
	if (servos.curr == servos.nb) {
		TimerLoadSet(TIMER1_BASE, TIMER_B, servos.remains);
		servos.remains = 20000UL; //load what remains of a 20ms pulse
		servos.curr = 0;
		GPIOPinWrite(LED_PORT, LED_B, 0);
	} else {
		GPIOPinWrite(LED_PORT, LED_B, LED_B);
		TimerLoadSet(TIMER1_BASE, TIMER_B,
				servos.servos[servos.curr].pulseWidth);
		servos.remains -= servos.servos[servos.curr].pulseWidth;
		GPIOPinWrite(servos.servos[servos.curr].port,
				servos.servos[servos.curr].pin, servos.servos[servos.curr].pin);
	}
	TimerEnable(TIMER1_BASE, TIMER_B);
}

