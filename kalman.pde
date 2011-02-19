#include <math.h>

static float kangle;
static float q_bias;
static float rate;

static float dt;

static float P[2][2] = {{1, 0},
				 {0, 1}};

// Seconds
void set_update_rate(float d) {
	dt = d;
}




