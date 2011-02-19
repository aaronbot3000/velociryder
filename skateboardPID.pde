#include <SoftwareSerial.h>

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

// Accelerometer center point
#define ACCL_CENTER 500   // units

// Balance adjust
#define MAXBALANCE 90  // units
#define MINBALANCE -90 // units
#define D_BALANCE 0.8 // units

#define GYRO_REDUC 0.33

// PID control constants
#define PGAIN 7.8
#define IGAIN 0 //0.03
#define DGAIN 2.1

#define INTEG_BUFFER_SIZE 128

// Other control constants
#define ACCL_MIX .005
#define MGAIN 1.6

//IMU constants
#define GYROTODEG4 3.0658682  // degrees per unit
#define GYROTODEG  0.7664671  // degrees per unit
#define ACCLTODEG  0.2661421  // degrees per unit

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

extern float angle;
extern float ygyro_bias;
extern float rate;

extern float ygyro_reading;
extern float zgyro_reading;
extern float accl_reading;
extern float turnpot_reading;

static float zgyro_bias;
static float turnpot_ref;
static float steer_req;

static float level;

static bool wait_for_level = false;

void setup() {
#ifdef OCCASIONALDEBUG
	Serial.begin(BAUD);
#endif

	init_sensors();
	init_motors();

	// Build up some previous values
	for (uint8_t i=0; i<200; i++) {
		run_magic();
	}

	// 5 seconds may seem long but it doesn't work too well unless its 5 seconds
	//delay (5000);

	read_zgyro();
	zgyro_bias = zgyro_reading;

	// Wait for user to level board
	wait_for_level = true;

	// the turnpot's value at center is different when at rest and when stood on
	// So take the reference point after the user is on the board
	read_turnpot();
	turnpot_ref = turnpot_reading;

	level = 0;
	angle = 0;
}

void process_steering() {
	float steer;
	float rot_rate;
	// steering here
	read_zgyro();
	read_turnpot();
	rot_rate = (zgyro_reading - zgyro_bias);
	steer = -(turnpot_reading - turnpot_ref);

	// If going straight
	if (abs(steer) <= TURNPOT_MARGIN) {
		if (abs(rot_rate) > STEER_GYRO_MARGIN)
			steer_req = STEER_CORRECT_POWER * rot_rate;
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



void run_magic() {
	static uint8_t filt_ind;
	static float filt_level[7];
	static unsigned long time = 0;

	float time_since = 0;
	p_angle = angle;
	accl = (read_accl() - ACCL_CENTER) * ACCLTODEG;

	time_since = (((float)(millis() - time))/1000.0);
	time = millis();

	ygyro = GYRO_REDUC * (read_ygyro() - ygyro_ref) * GYROTODEG * time_since;

	angle = ((angle + ygyro) * (1 - ACCL_MIX)) + (accl * ACCL_MIX);
	
	// P
	level = PGAIN * angle;

	// D
	level += DGAIN * (angle - p_angle);

	// Testing the Savitsky Golay filter for motor levels as well
	for (filt_ind=0; filt_ind<6; filt_ind++) {
		filt_level[filt_ind] = filt_level[filt_ind+1];
	}
	filt_level[6] = level;

	// Magic numbers!!!
	level = ((-2*filt_level[0]) + 
			 ( 3*filt_level[1]) + 
			 ( 6*filt_level[2]) + 
			 ( 7*filt_level[3]) + 
			 ( 6*filt_level[4]) + 
			 ( 3*filt_level[5]) + 
			 (-2*filt_level[6]))/21.0; 
}

void set_motors()
{
	int16_t motorL;
	int16_t motorR;
	motorL = MGAIN * level + steer_req;
	motorR = MGAIN * level - steer_req;

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
		Serial.print("gyro: ");
		Serial.println((read_ygyro()-ygyro_ref) * GYROTODEG);
		Serial.print("yaccl: ");
		Serial.println(accl);
		Serial.print("yaccl_raw: ");
		Serial.println(read_accl());
		Serial.print("ygyro: ");
		Serial.println(ygyro, 8);
		Serial.print("time: ");
		Serial.println(time_since, 8);
		Serial.print("steer: ");
		Serial.println(steer);
		Serial.print("steer_req: ");
		Serial.println(steer_req);
		Serial.print("wait level: ");
		Serial.println(wait_for_level);
		Serial.print("zgyro: ");
		Serial.println(zgyro);
		Serial.print("tpot: ");
		Serial.println(read_turnpot());
	}
}
#endif

