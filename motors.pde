// motor Controller defines
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

SoftwareSerial motors = SoftwareSerial(MOTOR_RX_PIN, MOTOR_TX_PIN);

void init_motors() {
	pinMode(OVERSPEED1, OUTPUT);
	pinMode(OVERSPEED2, OUTPUT);

	digitalWrite(OVERSPEED1, LOW);
	digitalWrite(OVERSPEED2, LOW);

	pinMode(MOTOR_TX_PIN, OUTPUT);
	motors.begin(SABER_BAUDRATE);
	delay(2000);
	motors.print(STOP, BYTE);
}

void signal_go_time() {
	// Signal we are good to go
	digitalWrite(OVERSPEED2, HIGH);
}

float softStartMult = 1;
void set_motors(int16_t motorL, int16_t motorR){
	// Activate the overspeed lights
	if (abs(motorL) > OVERSPEED_LEVEL_2 || abs(motorR) > OVERSPEED_LEVEL_2)
		digitalWrite(OVERSPEED2, LOW);
	if (abs(motorL) > OVERSPEED_LEVEL_1 || abs(motorR) > OVERSPEED_LEVEL_1)
		digitalWrite(OVERSPEED1, LOW);
	else {
		digitalWrite(OVERSPEED1, HIGH);
		digitalWrite(OVERSPEED2, HIGH);
	}

	// Slowly come back from a killswitch release
	if (softStartMult < 1)
		softStartMult += 0.01;

	if (!read_shit_switch()) {
		motorL = 0;
		motorR = 0;
		softStartMult = .2;
		motors.print(STOP, BYTE);
	}
	else {
		motorL *= softStartMult;
		motorR *= softStartMult;
		motorL = map(motorL, -100, 100, MOTOR1_MAX_BACK, MOTOR1_MAX_FRONT);
		motorR = map(motorR, -100, 100, MOTOR2_MAX_BACK, MOTOR2_MAX_FRONT);

		motors.print((uint8_t)motorL, BYTE);
		motors.print((uint8_t)motorR, BYTE);
	}
}
