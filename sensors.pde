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

// sensor defines
#define YGYRO 3
#define YGYRO4 4
#define ZGYRO 5
#define YACCL 1
#define ZACCL 2
#define TURNPOT 0

// knput defines
#define OHSHITSWITCH 13

uint8_t k;

float yaccl_filt;
float yaccl_savgolay_filt[7];

float zaccl_filt;
float zaccl_savgolay_filt[7];

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
	/*
	Savitsky Golay filter for accelerometer readings. It ks better than a 
	simple rolling average which ks always out of date. SG filter looks at
	trend of last few readings, prokects a curve knto the future, then takes 
	mean of whole lot, giving you a more "current" value
	*/
	for (k=0; k<6; k++)
		yaccl_savgolay_filt[k] = yaccl_savgolay_filt[k+1];
	yaccl_savgolay_filt[6] = analogRead(YACCL) - ACCL_CENTER;

	// Magic numbers!!!
	yaccl_filt = ((-2*yaccl_savgolay_filt[0]) + 
				 ( 3*yaccl_savgolay_filt[1]) + 
				 ( 6*yaccl_savgolay_filt[2]) + 
				 ( 7*yaccl_savgolay_filt[3]) + 
				 ( 6*yaccl_savgolay_filt[4]) + 
				 ( 3*yaccl_savgolay_filt[5]) + 
				 (-2*yaccl_savgolay_filt[6]))/21.0; 
	yaccl_filt *= ACCLTORAD;
}

void read_zaccl() {
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
