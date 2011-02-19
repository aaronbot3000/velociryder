/*
   Front of board
   -100 power
   +degree


   -zGyro   +zGyro
   Motor1   Motor2


   -degree
   +100 power
 */

#define DIV7      0.14285714285714 // answer to 1/7

// sensor defines
#define YGYRO 3
#define YGYRO4 4
#define ZGYRO 5
#define YACCL 1
#define ZACCL 2
#define TURNPOT 0

// input defines
#define OHSHITSWITCH 13

float yaccl_reading;
float zaccl_reading;

float ygyro_reading;
float zgyro_reading;

float turnpot_reading;

static uint8_t i;

void init_sensors() {
	analogReference(EXTERNAL);
}

void update_yaccl() {
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

void update_zaccl() {
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
	for (i=0; i<7; i++) {
		ygyro_reading += analogRead(YGYRO4);
	}
	ygyro_reading *= DIV7;
}

void read_zgyro() {
	zgyro_reading = 0;
	for (i=0; i<7; i++) {
		zgyro_reading += analogRead(ZGYRO);
	}
	zgyro_reading *= DIV7;
}

void read_turnpot() {
	turnpot_reading = 0;
	for (i=0; i<7; i++) {
		turnpot_reading += analogRead(TURNPOT);
	}
	turnpot_reading *= DIV7;
}
