// Motor Controller defines
#define MOTOR_TX_PIN 7
#define MOTOR_RX_PIN 6

// motor overspeed defines
#define OVERSPEED1   9
#define OVERSPEED2   8

#define SABER_BAUDRATE 9600

#define MOTOR1_MAX_FRONT 1
#define MOTOR1_MAX_BACK  127

#define MOTOR2_MAX_FRONT 128
#define MOTOR2_MAX_BACK  255

#define STOP 0

#define OVERSPEED_LEVEL_1 70
#define OVERSPEED_LEVEL_2 90

SoftwareSerial Motors = SoftwareSerial(MOTOR_RX_PIN, MOTOR_TX_PIN);

void init_motors() {
	pinMode(OVERSPEED1, OUTPUT);
	pinMode(OVERSPEED2, OUTPUT);

	digitalWrite(OVERSPEED1, LOW);
	digitalWrite(OVERSPEED2, LOW);

	pinMode(MOTOR_TX_PIN, OUTPUT);
	Motors.begin(SABER_BAUDRATE);
	delay(2000);
	Motors.print(STOP, BYTE);
}

float softStartMult = 1;

void set_motors(int16_t motor1, int16_t motor2){
	if (abs(motor1) > OVERSPEED_LEVEL_2 || abs(motor2) > OVERSPEED_LEVEL_2)
		digitalWrite(OVERSPEED2, LOW);
	if (abs(motor1) > OVERSPEED_LEVEL_1 || abs(motor2) > OVERSPEED_LEVEL_1)
		digitalWrite(OVERSPEED1, LOW);
	else {
		digitalWrite(OVERSPEED1, HIGH);
		digitalWrite(OVERSPEED2, HIGH);
	}

	if (softStartMult < 1)
		softStartMult += 0.01;

	if (!get_shit_switch()) {
		motor1 = 0;
		motor2 = 0;
		softStartMult = .2;
		Motors.print(STOP, BYTE);
	}

	else {
		motor1 *= softStartMult;
		motor2 *= softStartMult;
		motor1 = map(motor1, -100, 100, MOTOR1_MAX_BACK, MOTOR1_MAX_FRONT);
		motor2 = map(motor2, -100, 100, MOTOR2_MAX_BACK, MOTOR2_MAX_FRONT);

		Motors.print((uint8_t)motor1, BYTE);
		Motors.print((uint8_t)motor2, BYTE);
	}
}
