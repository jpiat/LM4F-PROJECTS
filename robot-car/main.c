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

#define SYSCTL_SONAR_PORT	SYSCTL_PERIPH_GPIOF
#define BASE_SONAR_PORT		GPIO_PORTF_BASE
#define SONAR_1_TRIG	GPIO_PIN_1
#define SONAR_1_ECHO	GPIO_PIN_2
#define SONAR_2_TRIG	GPIO_PIN_3
#define SONAR_2_ECHO	GPIO_PIN_4
#define SONAR_3_TRIG	GPIO_PIN_5
#define SONAR_3_ECHO	GPIO_PIN_6
#define SONAR_GPIO_OUTS (SONAR_1_TRIG | SONAR_2_TRIG)
#define SONAR_GPIO_INS  (SONAR_1_ECHO | SONAR_2_ECHO)

#define SYSCTL_SERVO_PORT	SYSCTL_PERIPH_GPIOC
#define BASE_SERVO_PORT		GPIO_PORTC_BASE
#define SERVO_1_PIN 	GPIO_PIN_1
#define SERVO_2_PIN 	GPIO_PIN_2
#define SERVO_GPIO_OUTS (SERVO_1_PIN | SERVO_2_PIN)
#define SERVO_MAX	90.0
#define SERVO_MIN	-90.0
#define SERVO_MAX_PULSE 2000UL
#define SERVO_MIN_PULSE	 1000UL
#define SERVO_PULSE_RANGE (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
#define SERVO_QUANTA		(SERVO_PULSE_RANGE/(SERVO_MAX - SERVO_MIN))
#define SERVO_CENTER	1500UL

unsigned char sonarFirstEdge = 0, pulseGen = 0;
unsigned long int sonarTemp = 0;

struct sonar_ranger {
	unsigned long trigPin;
	unsigned long echoPin;
	float lastDistance;
};

struct sonar_belt {
	unsigned char nb;
	unsigned char curent_measure;
	struct sonar_ranger ranger[8];
};

struct servo {
	unsigned long pin;
	unsigned long pulseWidth;
};

struct servo_array {
	unsigned char nb; //must be less than 8
	unsigned char curr;
	unsigned long int remains;
	struct servo servos[8];
};

struct sonar_belt ranger_belt;
struct servo_array servos;

// An interrupt function.
void Timer1A_ISR(void);
void Timer1B_ISR(void);
void GPIO_Sonar_ISR(void);
void initServos(struct servo_array * servosp, unsigned char nbServos);
void initBelt(struct sonar_belt * belt, unsigned char nbRangers);

// angle between -90 and 90
void setServoAngle(struct servo * servop, float angle) {
	unsigned long int pulse = 0;
	if (angle > SERVO_MAX) {
		servop->pulseWidth = SERVO_MAX_PULSE;
	} else if (angle < SERVO_MIN) {
		servop->pulseWidth = SERVO_MIN_PULSE;
	} else {
		pulse = angle - SERVO_MIN;
		pulse = pulse * SERVO_QUANTA;
		servop->pulseWidth = SERVO_MIN_PULSE + pulse;
	}
}

void initServos(struct servo_array * servosp, unsigned char nbServos) {
	unsigned int i;
	servosp->nb = nbServos;
	servosp->curr = 0;
	for (i = 0; i < servosp->nb; i++) {
		servosp->servos[i].pulseWidth = SERVO_CENTER; //center pos on all servos
	}
}

void initBelt(struct sonar_belt * belt, unsigned char nbRangers) {
	unsigned int i;
	belt->nb = nbRangers;
	belt->curent_measure = 0;
	for (i = 0; i < belt->nb; i++) {
		belt->ranger[i].lastDistance = 10;
	}
}

// main function.
int main(void) {
	unsigned int i=0, k=0;

	SysCtlClockSet(
			SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
					| SYSCTL_XTAL_16MHZ); //80Mhz

	initBelt(&ranger_belt, 2);
	ranger_belt.ranger[0].echoPin = SONAR_1_ECHO;
	ranger_belt.ranger[1].echoPin = SONAR_2_ECHO;
	ranger_belt.ranger[0].trigPin = SONAR_1_TRIG;
	ranger_belt.ranger[1].trigPin = SONAR_2_TRIG;

	initServos(&servos, 2);
	servos.servos[0].pin = SERVO_1_PIN;
	servos.servos[1].pin = SERVO_2_PIN;

	//GPIO_CONFIG: must config each sonar ranger port as gpio interrupt on BOTH_EDGE
	SysCtlPeripheralEnable(SYSCTL_SONAR_PORT);
	GPIOPinTypeGPIOOutput(BASE_SONAR_PORT, SONAR_GPIO_OUTS);
	GPIOPinWrite(BASE_SONAR_PORT, SONAR_GPIO_OUTS, 0); //clear all sonar pins
	GPIOPinTypeGPIOInput(BASE_SONAR_PORT, SONAR_GPIO_INS);
	GPIOIntTypeSet(BASE_SONAR_PORT, SONAR_GPIO_INS, GPIO_BOTH_EDGES);
	GPIOPortIntRegister(BASE_SONAR_PORT, GPIO_Sonar_ISR);

	//configuring servo pins as out
	SysCtlPeripheralEnable(SYSCTL_SERVO_PORT);
	GPIOPinTypeGPIOOutput(BASE_SERVO_PORT, SERVO_GPIO_OUTS);
	GPIOPinWrite(BASE_SERVO_PORT, SERVO_GPIO_OUTS, 0); //clear all servo pins

	//TIMER1_A_CONFIG : used for sonar pulses
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_ONE_SHOT); //one shot timer, half used
	TimerPrescaleSet(TIMER1_BASE, TIMER_A, 80); // set time prescaler to generate ~1us count
	TimerControlStall(TIMER1_BASE, TIMER_A, true);
	TimerLoadSet(TIMER1_BASE, TIMER_A, 15); //first pulse generation
	TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1A_ISR);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//TIME1_B_CONFIG : used for servo pulses
	TimerPrescaleSet(TIMER1_BASE, TIMER_B, 80); // set time prescaler to generate ~1us count
	TimerControlStall(TIMER1_BASE, TIMER_B, true);
	TimerLoadSet(TIMER1_BASE, TIMER_B, 20000UL); //first pulse generation
	TimerIntRegister(TIMER1_BASE, TIMER_B, Timer1B_ISR);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

	//generate first pulse to initiate measuring cycle
	pulseGen = 0;
	sonarFirstEdge = 1;
	GPIOPinWrite(BASE_SONAR_PORT, SONAR_1_TRIG, SONAR_1_TRIG); //set trig
	TimerEnable(TIMER1_BASE, TIMER_A);

	//
	// Initialize the UART
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//UARTStdioInitExpClk(0, 9600);  // Baud=9600,
	UARTStdioInit(0); //Baud = 115200

	while (1) {

		if (UARTCharsAvail(UART0_BASE)) {

		}
		if (ranger_belt.curent_measure == 0 && k != pulseGen && pulseGen == 0) { // on falling edge of pulseGen
			UARTprintf("[ ");
			for (i = 0; i < ranger_belt.nb; i++) {
				UARTprintf("%f", ranger_belt.ranger[i].lastDistance);
				if (i != (ranger_belt.nb - 1)) {
					UARTprintf(", ");
				} else {
					UARTprintf(" ");
				}
			}
			UARTprintf("]\r\n");

		}
		k = pulseGen ;
	}
}

//the GPIO interrupt handler
void GPIO_Sonar_ISR(void) {
	GPIOPinIntClear(BASE_SONAR_PORT, SONAR_GPIO_INS);
	//should be generated first on rising edge then falling edge
	unsigned long int valPins = GPIOPinRead(BASE_SONAR_PORT,
			ranger_belt.ranger[ranger_belt.curent_measure].echoPin); // check pin state
	if (sonarFirstEdge && valPins) { //rising edge on pin
		sonarTemp = TimerValueGet(TIMER1_BASE, TIMER_A);
		sonarFirstEdge = 0;
	} else if (!valPins) { // falling edge
		unsigned short int timerVal = TimerValueGet(TIMER1_BASE, TIMER_A);
		timerVal = timerVal - sonarTemp;
		ranger_belt.ranger[ranger_belt.curent_measure].lastDistance =
				((float) timerVal) / 58;
		GPIOPinIntDisable(BASE_SONAR_PORT,
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
		GPIOPinWrite(BASE_SONAR_PORT, currRanger.trigPin, currRanger.trigPin); //set trig
		pulseGen = 0;
	} else {
		GPIOPinWrite(BASE_SONAR_PORT,
				ranger_belt.ranger[ranger_belt.curent_measure].trigPin, 0); //clear trig
		GPIOPinIntEnable(BASE_SONAR_PORT,
				ranger_belt.ranger[ranger_belt.curent_measure].echoPin);
		sonarFirstEdge = 1;
		pulseGen = 1;
		TimerLoadSet(TIMER1_BASE, TIMER_A, 0xFFFF); // timer resolution is 1µs, counting 64ms
	}
	TimerEnable(TIMER1_BASE, TIMER_A);
}

void Timer1B_ISR(void) {
	TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
	GPIOPinWrite(BASE_SERVO_PORT, servos.servos[servos.curr].pin, 0);
	if (servos.remains != 20000UL) { //not a initial count ...
		servos.curr++;
	}
	if (servos.curr == servos.nb) {
		TimerLoadSet(TIMER1_BASE, TIMER_B, servos.remains);
		servos.remains = 20000UL; //load what remains of a 20ms pulse
		servos.curr = 0;
	} else {
		TimerLoadSet(TIMER1_BASE, TIMER_B,
				servos.servos[servos.curr].pulseWidth);
		servos.remains -= servos.servos[servos.curr].pulseWidth;
		GPIOPinWrite(BASE_SERVO_PORT, servos.servos[servos.curr].pin,
				servos.servos[servos.curr].pin);
	}
	TimerEnable(TIMER1_BASE, TIMER_B);
}

