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

// TODO: Add balance point shift compensation for acceleration and 
// deceleration. Check by seeing change kn level

// Changeables
//#define OCCASIONALDEBUG

/*
   Front of board
   -100 power
   +degree


   -zGyro   +zGyro
   Motor1   Motor2


   -degree
   +100 power
 */

// Gyro constants
// 572.958 mV per rad per sec, 1.7453292 rad/sec max
#define GYROTORAD4 .005624596 // radians per second-unit
// 143 mV per deg per sec, 6.981317007975 rad/sec max
#define GYROTORAD   .02249838 // radians per second-unit

// 400 at 0.392 radians
#define ACCLTORAD .00392

// Accelerometer center point
#define ACCL_CENTER 500   // units

#define YACCL_AVG_LEN 8

// sensor defines
#define YGYRO 3
#define YGYRO4 4
#define ZGYRO 5
#define YACCL 1
#define ZACCL 2
#define TURNPOT 0

// input defines
#define OHSHITSWITCH 13

static uint8_t k;

float yaccl_filt;
float zaccl_filt;

float ygyro4_sum;
float ygyro_sum;
float zgyro_sum;
float turnpot_sum;

void init_sensors() {
	analogReference(EXTERNAL);
}

bool read_shit_switch() {
	return digitalRead(OHSHITSWITCH);
}

void read_yaccl() {
	static int filt_ind = 0;
	static float sum = 0;
	static float yacclsum = 0;
	static float yaccl_filt[YACCL_AVG_LEN];

	yacclsum = 0;
	for (k=0; k<8; k++) {
		yacclsum += analogRead(YACCL) - ACCL_CENTER;
	}
	yacclsum /= 8;

	sum -= yaccl_filt[filt_ind];
	yaccl_filt[filt_ind] = yacclsum;
	sum += yaccl_filt[filt_ind];

	filt_ind = (++filt_ind) % YACCL_AVG_LEN;

	yaccl_filt = sum / YACCL_AVG_LEN;
	yaccl_filt *= ACCLTORAD;
}

void read_zaccl() {
	static float zaccl_savgolay_filt[7];
	// S-G filter again
	for (k=0; k<6; k++)
		zaccl_savgolay_filt[k] = zaccl_savgolay_filt[k+1];
	zaccl_savgolay_filt[6] = analogRead(ZACCL);

	// Magic numbers!!!
	zaccl_filt = ((-2*zaccl_savgolay_filt[0]) + 
				 ( 3*zaccl_savgolay_filt[1]) + 
				 ( 6*zaccl_savgolay_filt[2]) + 
				 ( 7*zaccl_savgolay_filt[3]) + 
				 ( 6*zaccl_savgolay_filt[4]) + 
				 ( 3*zaccl_savgolay_filt[5]) + 
				 (-2*zaccl_savgolay_filt[6]))/21.0; 
}

void read_ygyro4() {
	ygyro4_sum = 0;
	for (k=0; k<8; k++) {
		ygyro4_sum += analogRead(YGYRO4);
	}
	ygyro4_sum = ygyro4_sum / 8 * GYROTORAD4;
}

void read_ygyro() {
	ygyro_sum = 0;
	for (k=0; k<8; k++) {
		ygyro_sum += analogRead(YGYRO);
	}
	ygyro_sum = ygyro_sum / 8 * GYROTORAD;
}

void read_zgyro() {
	zgyro_sum = 0;
	for (k=0; k<8; k++) {
		zgyro_sum += analogRead(ZGYRO);
	}
	zgyro_sum = zgyro_sum / 8;
}

void read_turnpot() {
	turnpot_sum = 0;
	for (k=0; k<8; k++) {
		turnpot_sum += analogRead(TURNPOT);
	}
	turnpot_sum = turnpot_sum / 8;
}
