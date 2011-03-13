/*
Copyright (C) 2011 by Aaron Fan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// motor Controller defines
#define MOTOR_TX_PIN 11
#define MOTOR_RX_PIN 6

#define SABER_BAUDRATE 9600

#define MOTOR1_MAX_FRONT 1
#define MOTOR1_MAX_BACK  127

#define MOTOR2_MAX_FRONT 128
#define MOTOR2_MAX_BACK  255

#define STOP 0

SoftwareSerial motors = SoftwareSerial(MOTOR_RX_PIN, MOTOR_TX_PIN);
static float softStartMult = 1;

void init_motors() {
	pinMode(MOTOR_TX_PIN, OUTPUT);
	motors.begin(SABER_BAUDRATE);
	delay(2000);
	motors.print(STOP, BYTE);
}

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
	softStartMult = .7;
	motors.print(STOP, BYTE);
}

