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

#define MAXBALANCE 90  // units
#define MINBALANCE -90 // units

#define PGAIN .4 // *100 percent
#define IGAIN .4 // *100 percent
#define DGAIN .4 // *100 percent

//+ = tip forward
//- = tip backward
// Accelerometer center point
#define YCENTER 470   // units

// Not changeables
#define BAUD 9600

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

#define DIV7      0.14285714285714 // answer to 1/7
#define MAP_1024_TO_100(a) (((a) - 512) * 0.1953125)

// IMU defines
#define YGYRO 0
#define YGYRO4 1
#define ZGYRO 2
#define YACCL 3
#define TURNPOT 4

// Digital I/O defines
#define HEARTBEAT    13

float yAngle;

float steer_req;
float steer_correct;
int steer = 0;

float level = 0;
float levelSavGolayFilt[7];

float balance_trim;

#ifdef OCCASIONALDEBUG
char counter;
#endif

float cur_speed;
float level;
int   steer; 
float balance_trim;

void setup() {
#ifdef OCCASIONALDEBUG
	Serial.begin(BAUD);
#endif
	pinMode(HEARTBEAT, OUTPUT);

	init_sensors();
	init_motors();

	// Build up some previous values
	for (int i=0; i<200; i++) {
		next_motor_levels();
	}

	// 5 seconds may seem long but it doesn't work too well unless its 5 seconds
	delay (5000);

	ygyro_ref = read_ygyro();
	zgyro_ref = read_zgyro();

	// Signal we are good to go
	digitalWrite(OVERSPEED2, HIGH);

	// Wait for user to level board
	while (true) {
		for (int x=0; x<10; x++)
			next_motor_levels();
		if (abs(yAngle) < 3) {
			cur_speed = 0;
			level = 0;
			steer = 0;
			balance_trim = 0;
		}
		else
			break;
	}

	// the turnpot's value at center is different when at rest and when stood on
	// So take the reference point after the user is on the board
	turnpot_ref = read_turnpot();
	cur_speed = 0;
	steer_req = 512;
	steer_correct = 0;
	balance_trim = 0;
	level = 0;

	// Turn off the lights
	digitalWrite(OVERSPEED1, HIGH);
	digitalWrite(OVERSPEED2, HIGH);
}

void read_controls() {
	if (digitalRead(BALANCEUP))
		balance_trim += 0.08;
	if (digitalRead(BALANCEDOWN))
		balance_trim -= 0.08;
	balance_trim = constrain(balance_trim, MINBALANCE, MAXBALANCE);

	oh_shit_switch = digitalRead(OHSHITSWITCH);
}

void process_steering() {
	// steering here
	zGyroDegrees = -(zGyroSum - zgyro_ref) * GYROTODEG;

	// If going straight
	if (abs(turnPotSum - turnpot_ref) <= TURNPOTOFFSET)
		if (abs(zGyroDegrees) > STEERERROR)
			steer_correct = STEERCORRECTPOWER * zGyroDegrees;
		steer_req = 512;
	}

	else {
		float rawSteer = turnPotSum - turnpot_ref;

		if (turnPotSum < turnpot_ref)
			rawSteer += TURNPOTOFFSET;
		else
			rawSteer -= TURNPOTOFFSET;

		// Scale according to speed
		rawSteer *= STEERCONST * ((-abs(level) * (1-MINSTEER)/100) + 1);
		steer_req = 512 - rawSteer;
		steer_correct = 0;
	}
}

void next_motor_levels() {
	heartbeat();
	readInputs();
	processSteering();
	processAccelerometer();
	processGyroscope();

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
		Serial.print("YA: ");
		Serial.println(yAcclDegrees);
		Serial.print("YG: ");
		Serial.println(yGyroDegrees);
		Serial.print("cTime: ");
		Serial.println(CYCLE_TIME);
		Serial.print("magic: ");
		Serial.println(MAGIC_GYRO_MULT);
		Serial.print("YG: ");
		Serial.println(yGyroDegrees);
		Serial.print("level: ");
		Serial.println(level);
		Serial.print("btor: ");
		Serial.println(balanceTorque);
		Serial.print("yAnP2: ");
		Serial.println((ACCL_PROPORTION * yAcclDegrees));
		Serial.print("yAngle: ");
		Serial.println(yAngle);
		Serial.print("yAngle1: ");
		Serial.println((1-ACCL_PROPORTION) * (yAngle + yGyroDt));
		Serial.print("yGyroDt: ");
		Serial.println(yGyroDt);
		Serial.print("level: ");
		Serial.println(level);
		Serial.print("Motor1: ");
		Serial.println(motor1);
		Serial.print("Motor2: ");
		Serial.println(motor2);
		if (!oh_shit_switch)
		{
			Serial.println("Disabled");
		}
	}
}
#endif

signed int motor1;
signed int motor2;
void setMotors()
{
	steer = steer_correct + steer_req;  //at this point is on the 0-1023 scale 

	//steer_req is either 512 for dead ahead or bigger/smaller if you are pressing steering switch left or right
	//SteerCorrect is the "adjustment" made by the second gyro that resists sudden turns if one wheel hits a small object for example.
	steer = MAP_1024_TO_100(steer);   //gets it down from 0-1023 (with 512 as the middle no-steer point) to -100 to +100 with 0 as the middle no-steer point on scale
	steer = constrain(steer, -100, 100);

	motor1 = level - steer;
	motor2 = level + steer;

	motor1 = constrain(motor1, -100, 100);
	motor2 = constrain(motor2, -100, 100);

	motor1 *= softStartMult;
	motor2 *= softStartMult;


#ifdef OCCASIONALDEBUG
	printStatusToSerial();
#endif
}

unsigned char pulseCounterA = 0;
unsigned char pulseCounterB = 1;
unsigned char pulseCounterDir = -1;
unsigned char cycleCount = 0;
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
	getNextMotorLevels();
	setMotors();

#ifdef OCCASIONALDEBUG
	counter++;
	counter%=128;
#endif
}
