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

// sensor defines
#define YGYRO 0
#define YGYRO4 1
#define ZGYRO 2
#define YACCL 3
#define TURNPOT 4

void init() {
	analogReference(EXTERNAL);
}

float accl_filt;
float accl_savgolay_filt[7];
float read_accl() {
	// Savitsky Golay filter for accelerometer readings. It is better than a simple rolling average which is always out of date.
	// SG filter looks at trend of last few readings, projects a curve into the future, then takes mean of whole lot, giving you a more "current" value
	for (int x=0; x<6; x++)
		accl_savgolay_filt[x] = accl_savgolay_filt[x+1];
	accl_savgolay_filt[6] = analogRead(YACCL);

	// Magic numbers!!!
	accl_filt = ((-2*accl_savgolay_filt[0]) + 
				 ( 3*accl_savgolay_filt[1]) + 
				 ( 6*accl_savgolay_filt[2]) + 
				 ( 7*accl_savgolay_filt[3]) + 
				 ( 6*accl_savgolay_filt[4]) + 
				 ( 3*accl_savgolay_filt[5]) + 
				 (-2*accl_savgolay_filt[6]))/21.0; 

	return accl_filt;
}

float ygyro_sum;
float read_ygyro() {
	ygyro_sum = 0;
	for (int x=0; x<7; x++) {
		ygyro_sum += analogRead(YGYRO4);
	return ygyro_sum * DIV7;
}

float zgyro_sum;
float read_zgyro() {
	zgyro_sum = 0;
	for (int x=0; x<7; x++) {
		zgyro_sum += analogRead(ZGYRO);
	return zgyro_sum * DIV7;
}

float turnpot_sum;
float read_turnpot() {
	turnpot_sum = 0;
	for (int x=0; x<7; x++) {
		turnpot_sum += analogRead(TURNPOT);
	return turnpot_sum * DIV7;
}
