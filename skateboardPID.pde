#include <SoftwareSerial.h>

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
//+ = tip forward
//- = tip backward

// Accelerometer center point
#define YCENTER 470   // units

// Balance adjust
#define MAXBALANCE 90  // units
#define MINBALANCE -90 // units
#define D_BALANCE 0.8 // units

// PID control constants
#define PGAIN .4 // *100 percent
#define IGAIN .4 // *100 percent
#define DGAIN .4 // *100 percent

// Turning constants
#define TURNPOT_MARGIN 7
#define STEER_MARGIN 3
#define STEER_CORRECT_POWER .15
#define STEER_POWER .15
#define MIN_STEER 90

#define MAP_1024_TO_100(a) (((a) - 512) * 0.1953125)

// Digital I/O defines
#define HEARTBEAT    13

float yAngle;

float steer_correct;

#ifdef OCCASIONALDEBUG
#define BAUD 9600
uint8_t counter;
#endif

float steer_req;
float balance_trim;
float ygyro_ref;
float zgyro_ref;

void setup() {
	pinMode(HEARTBEAT, OUTPUT);

#ifdef OCCASIONALDEBUG
	Serial.begin(BAUD);
#endif

	init_sensors();
	init_motors();

	// Build up some previous values
	for (uint8_t i=0; i<200; i++) {
		next_motor_levels();
	}

	// 5 seconds may seem long but it doesn't work too well unless its 5 seconds
	delay (5000);

	ygyro_ref = read_ygyro();
	zgyro_ref = read_zgyro();

	signal_go_time();

	// Wait for user to level board
	while (true) {
		// TODO: implement
	}

	// the turnpot's value at center is different when at rest and when stood on
	// So take the reference point after the user is on the board
	turnpot_ref = read_turnpot();

	// Turn off the lights
	digitalWrite(OVERSPEED1, HIGH);
	digitalWrite(OVERSPEED2, HIGH);
}

void adj_balance() {
	balance_trim = constrain(balance_trim + (D_BALANCE * read_bal_switch()), MINBALANCE, MAXBALANCE);
}

float zgyro, steer;
void process_steering() {
	// steering here
	zgyro = -(read_zgyro() - zgyro_ref);
	steer = read_turnpot() - turnpot_ref;

	// If going straight
	if (abs(steer) <= TURNPOT_MARGIN)
		if (abs(zgyro) > STEER_MARGIN)
			steer_req = STEER_CORRECT_POWER * zgyro;
	}
	// We are turning
	else {
		if (turnPotSum < turnpot_ref)
			steer += TURNPOTOFFSET;
		else
			steer -= TURNPOTOFFSET;

		// Scale according to speed
		steer_req = steer * STEER_POWER * ((-abs(level) * (1-MIN_STEER)/100) + 1);
	}
}

void run_magic() {
	//TODO: implement PID control

	level = 

	// Testing the Savitsky Golay filter for motor levels as well
	for (int x=0; x<6; x++)
		levelSavGolayFilt[x] = levelSavGolayFilt[x+1];
	levelSavGolayFilt[6] = level;

	// Magic numbers!!!
	level = ((-2*levelSavGolayFilt[0]) + 
			 ( 3*levelSavGolayFilt[1]) + 
			 ( 6*levelSavGolayFilt[2]) + 
			 ( 7*levelSavGolayFilt[3]) + 
			 ( 6*levelSavGolayFilt[4]) + 
			 ( 3*levelSavGolayFilt[5]) + 
			 (-2*levelSavGolayFilt[6]))/21.0; 
}

#ifdef OCCASIONALDEBUG
void printStatusToSerial()
{
	if (counter == 127)
	{
	}
}
#endif

int16_t motorL;
int16_t motorR;
void set_motors()
{
	motorL = level - steer;
	motorR = level + steer;

	motorL = constrain(motorL, -100, 100);
	motorR = constrain(motorR, -100, 100);

	set_motors(motorL, motorR);

#ifdef OCCASIONALDEBUG
	printStatusToSerial();
#endif
}

uint8_t pulseCounterA = 0;
uint8_t pulseCounterB = 1;
uint8_t pulseCounterDir = -1;
uint8_t cycleCount = 0;
void heartbeat()
{
	// TODO: replace with a proper PWM blinky
	if (pulseCounterA % 50 == 0)
	{
		pulseCounterB += pulseCounterDir;
		if (pulseCounterB <= 1)
			pulseCounterDir = 1;
		if (pulseCounterB >= 5)
			pulseCounterDir = -1;
		pulseCounterA = 0;
	}

	if (cycleCount % pulseCounterB == 0)
		digitalWrite(HEARTBEAT, HIGH);
	else
		digitalWrite(HEARTBEAT, LOW);

	cycleCount++;
	if (cycleCount == 100)
		cycleCount = 0;
	pulseCounterA++;
}


void loop()
{
	run_magic();
	set_motors();

#ifdef OCCASIONALDEBUG
	counter++;
	counter%=128;
#endif
}
