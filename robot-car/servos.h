
#define SERVO_1_PORT	GPIO_PORTA_BASE
#define SERVO_2_PORT	GPIO_PORTA_BASE
#define SERVO_1_PIN 	GPIO_PIN_3
#define SERVO_2_PIN 	GPIO_PIN_2
#define SERVO_MAX	90.0
#define SERVO_MIN	-90.0
#define SERVO_MAX_PULSE 2000UL
#define SERVO_MIN_PULSE	 1000UL
#define SERVO_PULSE_RANGE (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
#define SERVO_QUANTA		(SERVO_PULSE_RANGE/(SERVO_MAX - SERVO_MIN))
#define SERVO_CENTER	1500UL



struct servo {
	unsigned long port;
	unsigned long pin;
	unsigned long pulseWidth;
};

struct servo_array {
	unsigned char nb; //must be less than 8
	unsigned char curr;
	unsigned long int remains;
	struct servo servos[8];
};

void initServos(struct servo_array * servosp, unsigned char nbServos) {
	unsigned int i;
	servosp->nb = nbServos;
	servosp->curr = 0;
	for (i = 0; i < servosp->nb; i++) {
		servosp->servos[i].pulseWidth = SERVO_CENTER; //center pos on all servos
	}
}

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

