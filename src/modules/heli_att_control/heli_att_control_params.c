/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_pos_control_params.c
 *
 * @brief Helicopter autonomous controller parameters
 *
 * The structure of this application will be the same as the default pixhawk controllers
 */

#include <px4_config.h>

#include <systemlib/param/param.h>

/**
 * Roll P gain
 *
 * Roll proportional gain, 
 * i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 100.0
 * @unit u
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_ROLL_KP, 0.0f);

/**
 * Roll D gain
 *
 * Roll derivative gain, i.e. gain on derivative in rad for error 1 rad/s.
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_ROLL_KD, 10.0f);

/**
 * Roll integral gain
 * 	
 * Roll gain for desired velocity and feedback
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_ROLL_KI, 0.0f);

/**
 * Roll position gain
 * 	
 * Roll gain for desired velocity and feedback
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_ROLL_KX, 0.0f);

/**
 * Roll offset
 * 	
 * Added to output to level blades
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_ROLL_OFF, 0.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_PITCH_KP, 0.0f);

/**
 * Pitch D gain
 *
 * Pitch derivative gain, i.e. gain on derivative in rad for error 1 rad/s.
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_PITCH_KD, 0.0f);

/**
 * Pitch integral (velocity) gain
 * 	
 * For inverse plant
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_PITCH_KI, 0.0f);

/**
 * Pitch position gain
 * 	
 * For inverse plant
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_PITCH_KX, 0.0f);

/**
 * Pitch offset
 * 	
 * Added to output to level blades
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */


PARAM_DEFINE_FLOAT(HELI_PITCH_OFF, 0.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_YAW_KP, 0.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_YAW_KD, .05f);


/**
 * Yaw offset
 * 	
 * Added to output to level blades
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */


PARAM_DEFINE_FLOAT(HELI_YAW_OFF, 0.0f);

/**
 * Collective gain
 *
 * Proportional gain on collective
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_COLL_KP, 0.0f);

/**
 * Pitch D gain
 *
 * Collective derivative gain, i.e. gain on derivative in rad for error 1 rad/s.
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_COLL_KD, 0.0f);

/**
 * Collective stability damping
 * 	
 * For inverse plant
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_COLL_OFF, 0.0f);

/**
 * Collective open loop gain
 * 	
 * Only used in the open loop control on the heave channel. 
 *
 * @min 0.0
 * @group Helicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(HELI_COLL_SCALE, 0.0f);

PARAM_DEFINE_FLOAT(HELI_ROLLV_LIM, 0.0f);

PARAM_DEFINE_FLOAT(HELI_ROLLX_LIM, 0.0f);

PARAM_DEFINE_FLOAT(HELI_PITCHV_LIM, 0.0f);

PARAM_DEFINE_FLOAT(HELI_PITCHX_LIM, 0.0f);

PARAM_DEFINE_FLOAT(HELI_TAIL_ON, 0.0f);



	