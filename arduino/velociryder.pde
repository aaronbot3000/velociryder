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


#include <SoftwareSerial.h>
#include <math.h>

#define OCCASIONALDEBUG

/*
   Front of board
   +100 power
   +degree


   -zGyro   +zGyro
   Motor1   Motor2


   -degree
   -100 power
 */
//+ = tip forward
//- = tip backward


// PID control constants
// Can change by around 5 at a time
#define PGAIN 650
#define DGAIN 450

#define P3MULT 135

#define LVL_AVG_LEN 8

// Other control constants
#define ACCL_MIX .008

// Turning constants
#define TURNPOT_MARGIN 28
#define STEER_OFFSET 0
#define STEER_GYRO_MARGIN 8
#define STEER_CORRECT_POWER .30
#define STEER_POWER .20
#define MIN_STEER 90

#ifdef OCCASIONALDEBUG
#define BAUD 57600
static uint8_t counter;
#endif
static uint8_t i, j;

static float steer_req;

static float ygyro4_ref;
static float ygyro_ref;
static float zgyro_ref;
static float turnpot_ref;

static float level;
static float angle = 0;

static bool wait_for_level = false;

extern float yaccl_filt;
extern float zaccl_filt;
extern float ygyro4_sum;
extern float ygyro_sum;
extern float zgyro_sum;
extern float turnpot_sum;

void setup() {
#ifdef OCCASIONALDEBUG
	Serial.begin(BAUD);
#endif

	init_sensors();
	init_motors();

	// Build up some previous values
	for (int i=0; i<200; i++) {
		run_magic();
	}

	// 5 seconds may seem long but it doesn't work too well unless its 5 seconds
	//delay (5000);

	read_ygyro4();
	read_ygyro();
	read_zgyro();

	ygyro4_ref = ygyro4_sum;
	ygyro_ref = ygyro_sum;
	zgyro_ref = zgyro_sum;

	// Wait for user to level board
	wait_for_level = true;


	// the turnpot's value at center is different when at rest and when stood on
	// So take the reference point after the user is on the board
	read_turnpot();
	turnpot_ref = turnpot_sum;

	level = 0;
	angle = 0;
}

void process_steering() {
	float zgyro, steer;

	read_turnpot();
	steer = -(turnpot_sum - turnpot_ref);

	// If going straight
	if (abs(steer) <= TURNPOT_MARGIN) {
		read_zgyro();
		zgyro = (zgyro_sum - zgyro_ref);

		if (abs(zgyro) > STEER_GYRO_MARGIN)
			steer_req = STEER_CORRECT_POWER * zgyro;
		else
			steer_req = 0;
	}
	// We are turning
	else {
		if (steer < turnpot_ref)
			steer += STEER_OFFSET;
		else
			steer -= STEER_OFFSET;

		// Scale according to speed
		steer_req = steer * STEER_POWER ;//* ((-abs(level) * (1-MIN_STEER)/100) + 1);
	}
}

float accl_angle;

void run_magic() {
	static float filt_level[LVL_AVG_LEN];
	static float sum;
	static int filt_ind;

	static float p_angle = 0;
	static unsigned long time = 0;

	float gyro;
	float time_since = 0;
	float avg = 0;

	p_angle = angle;

	time_since = (((float)(millis() - time))/1000.0);
	time = millis();

#ifdef OCCASIONALDEBUG
	if (counter == 0) {
		Serial.print("Time: ");
		Serial.println(time_since, 10);
	}
#endif

	read_ygyro4();
	read_yaccl();

	if (abs(ygyro4_sum - ygyro4_ref) < 1.5) {
		read_ygyro();
		gyro = (ygyro_sum - ygyro_ref) * time_since;
	}
	else {
		gyro = (ygyro4_sum - ygyro4_ref) * time_since;
	}

	//accl_angle = atan2(yaccl_filt - ACCL_CENTER, zaccl_filt - ACCL_CENTER);
	accl_angle = yaccl_filt;

	angle = ((angle + gyro) * (1 - ACCL_MIX)) + (accl_angle * ACCL_MIX);

	// P
	level = PGAIN * (P3MULT * angle * angle * angle + angle);

	// D
	level += DGAIN * (angle - p_angle);

	sum -= filt_level[filt_ind];
	filt_level[filt_ind] = level;
	sum += filt_level[filt_ind];

	filt_ind = (++filt_ind) % LVL_AVG_LEN;
	
	level = sum / LVL_AVG_LEN;
}

void set_motors()
{
	int16_t motorL = level + steer_req;
	int16_t motorR = level - steer_req;

	motorL = constrain(motorL, -100, 100);
	motorR = constrain(motorR, -100, 100);

	if (!read_shit_switch()) {
		kill_motors();
		wait_for_level = true;
	}

	else if (wait_for_level) {
		if (abs(level) < 10)
			wait_for_level = false;
	}

	else {
		send_motor_command(motorL, motorR);
	}

#ifdef OCCASIONALDEBUG
	printStatusToSerial();
#endif
}

void loop()
{
	run_magic();
	process_steering();
	set_motors();

#ifdef OCCASIONALDEBUG
	counter++;
	counter%=64;
#endif
}

#ifdef OCCASIONALDEBUG
void printStatusToSerial()
{
	if (counter == 0)
	{
		Serial.println("=========");
		Serial.print("lvl: ");
		Serial.println(level);
		Serial.print("ang: ");
		Serial.println(angle);
		Serial.print("accl_angle: ");
		Serial.println(accl_angle);
		Serial.print("gyro4: ");
		Serial.println((ygyro4_sum - ygyro4_ref), 8);
		Serial.print("gyro: ");
		Serial.println((ygyro_sum - ygyro_ref), 8);
		Serial.print("yaccl: ");
		Serial.println(yaccl_filt);
		Serial.print("zaccl: ");
		Serial.println(zaccl_filt);
		Serial.print("steer: ");
		Serial.println(turnpot_sum - turnpot_ref);
		Serial.print("steer_req: ");
		Serial.println(steer_req);
		Serial.print("wait level: ");
		Serial.println(wait_for_level);
		Serial.print("zgyro: ");
		Serial.println(zgyro_sum);
	}
}
#endif

