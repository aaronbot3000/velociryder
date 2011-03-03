// TODO: Add balance point shift compensation for acceleration and 
// deceleration. Check by seeing change in level

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

#define DIV7      0.14285714285714 // answer to 1/7

//IMU constants
//ARef at 3.34 V
// 100 deg per second
// 2.5 mV per (deg per second)
#define GYROTODEG 1.3046875  // degrees per unit

// 400 deg per second
// 10 mV per (deg per second)
#define GYROTODEG4 3.26171875  // degrees per unit

// .971V at 45 degrees
// 2.074 at 135 degrees
#define ACCLTODEG .26614205575702629  // degrees per unit

// sensor defines
#define YGYRO 3
#define YGYRO4 4
#define ZGYRO 5
#define YACCL 1
#define ZACCL 2
#define TURNPOT 0

// input defines
#define OHSHITSWITCH 13

uint8_t i;

void init_sensors() {
	analogReference(EXTERNAL);
}

float y_accl_filt;
float y_accl_savgolay_filt[7];
float read_y_accl() {
	// Savitsky Golay filter for accelerometer readings. It is better than a simple rolling average which is always out of date.
	// SG filter looks at trend of last few readings, projects a curve into the future, then takes mean of whole lot, giving you a more "current" value
	for (i=0; i<6; i++)
		y_accl_savgolay_filt[i] = y_accl_savgolay_filt[i+1];
	y_accl_savgolay_filt[6] = analogRead(YACCL);

	// Magic numbers!!!
	y_accl_filt = ((-2*y_accl_savgolay_filt[0]) + 
				 ( 3*y_accl_savgolay_filt[1]) + 
				 ( 6*y_accl_savgolay_filt[2]) + 
				 ( 7*y_accl_savgolay_filt[3]) + 
				 ( 6*y_accl_savgolay_filt[4]) + 
				 ( 3*y_accl_savgolay_filt[5]) + 
				 (-2*y_accl_savgolay_filt[6]))/21.0; 

	return y_accl_filt;
}

float z_accl_filt;
float z_accl_savgolay_filt[7];
float read_z_accl() {
	// Savitsky Golay filter for accelerometer readings. It is better than a simple rolling average which is always out of date.
	// SG filter looks at trend of last few readings, projects a curve into the future, then takes mean of whole lot, giving you a more "current" value
	for (i=0; i<6; i++)
		z_accl_savgolay_filt[i] = z_accl_savgolay_filt[i+1];
	z_accl_savgolay_filt[6] = analogRead(YACCL);

	// Magic numbers!!!
	z_accl_filt = ((-2*z_accl_savgolay_filt[0]) + 
				 ( 3*z_accl_savgolay_filt[1]) + 
				 ( 6*z_accl_savgolay_filt[2]) + 
				 ( 7*z_accl_savgolay_filt[3]) + 
				 ( 6*z_accl_savgolay_filt[4]) + 
				 ( 3*z_accl_savgolay_filt[5]) + 
				 (-2*z_accl_savgolay_filt[6]))/21.0; 

	return z_accl_filt;
}

bool read_shit_switch() {
	return digitalRead(OHSHITSWITCH);
}

float ygyro_sum;
float read_ygyro() {
	ygyro_sum = 0;
	for (i=0; i<7; i++) {
		ygyro_sum += analogRead(YGYRO4);
	}
	return ygyro_sum * DIV7;
}

float zgyro_sum;
float read_zgyro() {
	zgyro_sum = 0;
	for (i=0; i<7; i++) {
		zgyro_sum += analogRead(ZGYRO);
	}
	return zgyro_sum * DIV7;
}

float turnpot_sum;
float read_turnpot() {
	turnpot_sum = 0;
	for (i=0; i<7; i++) {
		turnpot_sum += analogRead(TURNPOT);
	}
	return turnpot_sum * DIV7;
}
