/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: tilt.c,v 1.1 2003/07/09 18:23:29 john Exp $
 *
 * 1 dimensional tilt sensor using a dual axis accelerometer
 * and single axis angular rate gyro.  The two sensors are fused
 * via a two state Kalman filter, with one state being the angle
 * and the other state being the gyro bias.
 *
 * Gyro bias is automatically tracked by the filter.  This seems
 * like magic.
 *
 * Please note that there are lots of comments in the functions and
 * in blocks before the functions.  Kalman filtering is an already complex
 * subject, made even more so by extensive hand optimizations to the C code
 * that implements the filter.  I've tried to make an effort of explaining
 * the optimizations, but feel free to send mail to the mailing list,
 * autopilot-devel@lists.sf.net, with questions about this code.
 *
 * 
 * (c) 2003 Trammell Hudson <hudson@rotomotion.com>
 *
 *************
 *
 *  This file is part of the autopilot onboard code package.
 *  
 *  Autopilot is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *  
 *  Autopilot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with Autopilot; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Modified for Arduino use by Aaron Fan
 */

#include <math.h>

/*
 * R represents the measurement covariance noise.  In this case,
 * it is a 1x1 matrix that says that we expect 0.3 rad jitter
 * from the accelerometer.
 */
#define R_ANGLE 0.3

/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the acceleromter
 * relative to the gyros.
 */
#define Q_ANGLE 0.001
#define Q_GYRO  0.003
 
/*
 * Our two states, the angle and the gyro bias.  As a byproduct of computing
 * the angle, we also have an unbiased angular rate available.   These are
 * read-only to the user of the module.
 */
float angle;
float ygyro_bias;
float rate;

/*
 * Our update rate.  This is how often our state is updated with
 * gyro rate measurements.  For now, we do it every time an
 * 8 bit counter running at CLK/1024 expires.  You will have to
 * change this value if you update at a different rate.
 *
 * THIS IS IN SECONDS
 */
static float dt;

/*
 * Our covariance matrix.  This is updated at every time step to
 * determine how well the sensors are tracking the actual state.
 */
static float P[2][2] = {{1, 0},
				 {0, 1}};

// Seconds
void set_update_rate(float d) {
	dt = d;
}

/*
 * state_update is called every dt with a biased gyro measurement
 * by the user of the module.  It updates the current angle and
 * rate estimate.
 *
 * The pitch gyro measurement should be scaled into real units, but
 * does not need any bias removal.  The filter will track the bias.
 *
 * Our state vector is:
 *
 *	X = [ angle, gyro_bias ]
 *
 * It runs the state estimation forward via the state functions:
 *
 *	Xdot = [ angle_dot, gyro_bias_dot ]
 *
 *	angle_dot	= gyro - gyro_bias
 *	gyro_bias_dot	= 0
 *
 * And updates the covariance matrix via the function:
 *
 *	Pdot = A*P + P*A' + Q
 *
 * A is the Jacobian of Xdot with respect to the states:
 *
 *	A = [ d(angle_dot)/d(angle)     d(angle_dot)/d(gyro_bias) ]
 *	    [ d(gyro_bias_dot)/d(angle) d(gyro_bias_dot)/d(gyro_bias) ]
 *
 *	  = [ 0 -1 ]
 *	    [ 0  0 ]
 *
 * Due to the small CPU available on the microcontroller, we've
 * hand optimized the C code to only compute the terms that are
 * explicitly non-zero, as well as expanded out the matrix math
 * to be done in as few steps as possible.  This does make it harder
 * to read, debug and extend, but also allows us to do this with
 * very little CPU time.
 */

// Takes in pitch gyro measurement
void state_update(const float ygyro_m) {
	const float ygyro = ygyro_m - ygyro_bias;

}
