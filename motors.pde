// motor Controller defines
#define MOTOR_TX_PIN 11
#define MOTOR_RX_PIN 6

// motor overspeed defines
#define SABER_BAUDRATE 9600

#define MOTOR1_MAX_FRONT 1
#define MOTOR1_MAX_BACK  127

#define MOTOR2_MAX_FRONT 128
#define MOTOR2_MAX_BACK  255

#define STOP 0

SoftwareSerial motors = SoftwareSerial(MOTOR_RX_PIN, MOTOR_TX_PIN);

void init_motors() {
	pinMode(MOTOR_TX_PIN, OUTPUT);
	motors.begin(SABER_BAUDRATE);
	delay(2000);
	motors.print(STOP, BYTE);
}

float softStartMult = 1;
void send_motor_command(int16_t motorL, int16_t motorR) {
	// Slowly come back from a killswitch release
	if (softStartMult < 1)
		softStartMult += 0.01;

	motorL *= softStartMult;
	motorR *= softStartMult;
	motorL = map(motorL, 100, -100, MOTOR1_MAX_BACK, MOTOR1_MAX_FRONT);
	motorR = map(motorR, 100, -100, MOTOR2_MAX_BACK, MOTOR2_MAX_FRONT);

	motors.print((uint8_t)motorL, BYTE);
	motors.print((uint8_t)motorR, BYTE);
}

void kill_motors() {
	motorL = 0;
	motorR = 0;
	softStartMult = .7;
	motors.print(STOP, BYTE);
}

