/*
   Front of board
   -100 power
   +degree


   -zGyro   +zGyro
   Motor1   Motor2


   -degree
   +100 power
 */

// sensor defines
#define YGYRO 3
#define YGYRO4 4
#define ZGYRO 5
#define YACCL 1
#define ZACCL 2
#define TURNPOT 0

// input defines
#define OHSHITSWITCH 13

// Gyro constants
// 572.958 mV per rad per sec, 1.7453292 rad/sec max
#define GYROTORAD4 .001846446 // radians per unit
// 143 mV per deg per sec, 6.981317007975 rad/sec max
#define GYROTORAD .0004608398  // degrees per unit

float yaccl_reading;
float zaccl_reading;

float ygyro_reading;
float zgyro_reading;

float turnpot_reading;

static uint8_t i;

void init_sensors() {
	analogReference(EXTERNAL);
}

void read_yaccl() {
	static float savgolay_filt[7];
	// Savitsky Golay filter for accelerometer readings. It is better than a simple rolling average which is always out of date.
	// SG filter looks at trend of last few readings, projects a curve into the future, then takes mean of whole lot, giving you a more "current" value
	for (i=0; i<6; i++)
		savgolay_filt[i] = savgolay_filt[i+1];
	savgolay_filt[6] = analogRead(YACCL);

	// Magic numbers!!!
	yaccl_reading = ((-2*savgolay_filt[0]) + 
				 ( 3*savgolay_filt[1]) + 
				 ( 6*savgolay_filt[2]) + 
				 ( 7*savgolay_filt[3]) + 
				 ( 6*savgolay_filt[4]) + 
				 ( 3*savgolay_filt[5]) + 
				 (-2*savgolay_filt[6]))/21.0; 
}

void read_zaccl() {
	static float savgolay_filt[7];
	// Savitsky Golay filter for accelerometer readings. It is better than a simple rolling average which is always out of date.
	// SG filter looks at trend of last few readings, projects a curve into the future, then takes mean of whole lot, giving you a more "current" value
	for (i=0; i<6; i++)
		savgolay_filt[i] = savgolay_filt[i+1];
	savgolay_filt[6] = analogRead(ZACCL);

	// Magic numbers!!!
	zaccl_reading = ((-2*savgolay_filt[0]) + 
				 ( 3*savgolay_filt[1]) + 
				 ( 6*savgolay_filt[2]) + 
				 ( 7*savgolay_filt[3]) + 
				 ( 6*savgolay_filt[4]) + 
				 ( 3*savgolay_filt[5]) + 
				 (-2*savgolay_filt[6]))/21.0; 
}

bool read_shit_switch() {
	return digitalRead(OHSHITSWITCH);
}

void read_ygyro() {
	ygyro_reading = 0;
	for (i=0; i<8; i++) {
		ygyro_reading += analogRead(YGYRO4);
	}
	ygyro_reading = ygyro_reading / 8 * GYROTORAD4;
}

void read_ygyro_large_range() {
	ygyro_reading = 0;
	for (i=0; i<8; i++) {
		ygyro_reading += analogRead(YGYRO);
	}
	ygyro_reading = ygyro_reading / 8 * GYROTORAD;
}

void read_zgyro() {
	zgyro_reading = 0;
	for (i=0; i<8; i++) {
		zgyro_reading += analogRead(ZGYRO);
	}
	zgyro_reading /= 8;
}

void read_turnpot() {
	turnpot_reading = 0;
	for (i=0; i<8; i++) {
		turnpot_reading += analogRead(TURNPOT);
	}
	turnpot_reading /= 8;
}
