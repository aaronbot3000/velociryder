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

// Accelerometer center point
#define ACCL_CENTER 500   // units

// PID control constants
// Can change by around 5 at a time
#define PGAIN 180
#define DGAIN 60 //120.321

// Other control constants
#define ACCL_MIX .005

// Turning constants
#define TURNPOT_MARGIN 28
#define STEER_OFFSET 0
#define STEER_GYRO_MARGIN 8
#define STEER_CORRECT_POWER .30
#define STEER_POWER .20
#define MIN_STEER 90

#ifdef OCCASIONALDEBUG
#define BAUD 57600
uint8_t counter;
#endif

float steer_req;
float balance_trim;
float ygyro4_ref;
float ygyro_ref;
float zgyro_ref;
float turnpot_ref;
float level;
float angle = 0;

bool wait_for_level = false;

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

	ygyro4_ref = read_ygyro4();
	ygyro_ref = read_ygyro();
	zgyro_ref = read_zgyro();

	// Wait for user to level board
	wait_for_level = true;

	// the turnpot's value at center is different when at rest and when stood on
	// So take the reference point after the user is on the board
	turnpot_ref = read_turnpot();

	level = 0;
	angle = 0;
}

float zgyro, steer;
void process_steering() {
	// steering here
	zgyro = (read_zgyro() - zgyro_ref);
	steer = -(read_turnpot() - turnpot_ref);

	// If going straight
	if (abs(steer) <= TURNPOT_MARGIN) {
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

uint8_t filt_ind;
float filt_level[7];

float p_angle = 0;
float ygyro4;
float accl_angle;

float time_since = 0;
unsigned long time = 0;

void run_magic() {
	p_angle = angle;

	time_since = (((float)(millis() - time))/1000.0);
	time = millis();

	ygyro4 = (read_ygyro4() - ygyro4_ref) * time_since;
	
	accl_angle = atan2(read_yaccl() - ACCL_CENTER, read_zaccl() - ACCL_CENTER);

	angle = ((angle + ygyro4) * (1 - ACCL_MIX)) + (accl_angle * ACCL_MIX);
	
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

int16_t motorL;
int16_t motorR;
void set_motors()
{
	motorL = level + steer_req;
	motorR = level - steer_req;

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
		Serial.print("gyro4: ");
		Serial.println((read_ygyro4()-ygyro4_ref), 8);
		Serial.print("gyro: ");
		Serial.println((read_ygyro()-ygyro_ref), 8);
		Serial.print("yaccl: ");
		Serial.println(accl_angle);
		Serial.print("yaccl_raw: ");
		Serial.println(read_yaccl());
		Serial.print("ygyro4: ");
		Serial.println(ygyro4, 8);
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

