
#define SONAR_1_TRIG_PORT GPIO_PORTA_BASE
#define SONAR_1_ECHO_PORT GPIO_PORTF_BASE
#define SONAR_1_TRIG	GPIO_PIN_7
#define SONAR_1_ECHO	GPIO_PIN_1

#define SONAR_2_TRIG_PORT GPIO_PORTA_BASE
#define SONAR_2_ECHO_PORT GPIO_PORTE_BASE
#define SONAR_2_TRIG	GPIO_PIN_6
#define SONAR_2_ECHO	GPIO_PIN_3

#define SONAR_3_TRIG_PORT GPIO_PORTA_BASE
#define SONAR_3_ECHO_PORT GPIO_PORTE_BASE
#define SONAR_3_TRIG	GPIO_PIN_5
#define SONAR_3_ECHO	GPIO_PIN_2

#define SONAR_4_TRIG_PORT GPIO_PORTB_BASE
#define SONAR_4_ECHO_PORT GPIO_PORTE_BASE
#define SONAR_4_TRIG	GPIO_PIN_4
#define SONAR_4_ECHO	GPIO_PIN_1

#define SONAR_5_TRIG_PORT GPIO_PORTE_BASE
#define SONAR_5_ECHO_PORT GPIO_PORTD_BASE
#define SONAR_5_TRIG	GPIO_PIN_5
#define SONAR_5_ECHO	GPIO_PIN_3

#define SONAR_6_TRIG_PORT GPIO_PORTE_BASE
#define SONAR_6_ECHO_PORT GPIO_PORTD_BASE
#define SONAR_6_TRIG	GPIO_PIN_4
#define SONAR_6_ECHO	GPIO_PIN_2



struct sonar_ranger {
	unsigned long trigPort;
	unsigned long trigPin;
	unsigned long echoPort;
	unsigned long echoPin;
	float heading;
	float cone;
	float lastDistance;
};

struct sonar_belt {
	unsigned char nb;
	unsigned char curent_measure;
	struct sonar_ranger ranger[8];
};

void initBelt(struct sonar_belt * belt, unsigned char nbRangers) {
	unsigned int i;
	belt->nb = nbRangers;
	belt->curent_measure = 0;
	for (i = 0; i < belt->nb; i++) {
		belt->ranger[i].lastDistance = -1;
	}
}
