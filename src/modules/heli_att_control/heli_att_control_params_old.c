// /****************************************************************************
//  *
//  *  Arjun Bhargava
//  *	Otherlab 2015
//  *
//  ****************************************************************************/

// /**
//  * @file heli_pos_control_params.c
//  *
//  * @brief Helicopter autonomous controller parameters
//  *
//  * The structure of this application will be the same as the default pixhawk controllers
//  */

// #include <px4_config.h>

// #include <systemlib/param/param.h>

// /**
//  * Roll P gain
//  *
//  * Roll proportional gain, 
//  * i.e. desired angular speed in rad/s for error 1 rad.
//  *
//  * @min 0.0
//  * @max 100.0
//  * @unit u
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_ROLL_P, 0.0f);

// /**
//  * Roll D gain
//  *
//  * Roll derivative gain, i.e. gain on derivative in rad for error 1 rad/s.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_ROLL_D, 10.0f);

// /**
//  * Roll integral gain
//  * 	
//  * Roll gain for desired velocity and feedback
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_ROLL_I, 0.0f);

// /**
//  * Roll position gain
//  * 	
//  * Roll gain for desired velocity and feedback
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_ROLL_POS, 0.0f);
// /**
//  * Roll stability damping
//  * 	
//  * For inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_ROLL_LP, 0.0f);

// /**
//  * Roll control power for inverse plant
//  * 	
//  * Control power for inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_ROLL_CP, 0.0f);

// /**
//  * Roll offset
//  * 	
//  * Added to output to level blades
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */


// PARAM_DEFINE_FLOAT(HELI_ROLL_OFF, 0.0f);

// /**
//  * Pitch P gain
//  *
//  * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_PITCH_P, 0.0f);

// /**
//  * Pitch D gain
//  *
//  * Pitch derivative gain, i.e. gain on derivative in rad for error 1 rad/s.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_PITCH_D, 0.0f);

// /**
//  * Pitch integral (velocity) gain
//  * 	
//  * For inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_PITCH_I, 0.0f);

// /**
//  * Pitch position gain
//  * 	
//  * For inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_PITCH_POS, 0.0f);

// /**
//  * Pitch stability damping
//  * 	
//  * For inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */


// PARAM_DEFINE_FLOAT(HELI_PITCH_MQ, 0.0f);

// /**
//  * Pitch control power for inverse plant
//  * 	
//  * Control power for inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_PITCH_CP, 0.0f);

// /**
//  * Pitch offset
//  * 	
//  * Added to output to level blades
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */


// PARAM_DEFINE_FLOAT(HELI_PITCH_OFF, 0.0f);

// *
//  * Yaw P gain
//  *
//  * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
 



// PARAM_DEFINE_FLOAT(HELI_YAW_P, 0.0f);

// /**
//  * Yaw rate P gain
//  *
//  * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_YAW_D, .05f);


// /**
//  * Yaw offset
//  * 	
//  * Added to output to level blades
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */


// PARAM_DEFINE_FLOAT(HELI_YAW_OFF, 0.0f);

// /**
//  * Yaw stability damping
//  * 	
//  * For inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_YAW_NR, 0.0f);

// /**
//  * Yaw control power for inverse plant
//  * 	
//  * Control power for inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_YAW_CP, 0.0f);

// /**
//  * Yaw cross feed gain
//  *
//  * Added to output, crossfeed with collective input (thrust)
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_YAW_CF, 0.0f);

// /**
//  * Yaw washout gain
//  *
//  * Added to output, crossfeed with collective input (thrust)
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */


// PARAM_DEFINE_FLOAT(HELI_YAW_WASH, 1.0f);


// /**
//  * Roll Rate Max
//  *
//  * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_ROLL_SCALE, 1.0f);


// /**
//  * Pitch Rate Max
//  *
//  * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_PITCH_SCALE, 1.0f);


// /**
//  * Yaw Rate Max
//  *
//  * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */
// PARAM_DEFINE_FLOAT(HELI_YAW_SCALE, 1.0f);


// /**
//  * Collective gain
//  *
//  * Scaling for the gain on the collective
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_COLL_SCALE, 0.5f);

// /**
//  * Collective P gain
//  *
//  * Collective proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_COLL_P, 0.0f);

// /**
//  * Pitch D gain
//  *
//  * Collective derivative gain, i.e. gain on derivative in rad for error 1 rad/s.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_COLL_D, 0.0f);

// /**
//  * Collective stability damping
//  * 	
//  * For inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_COLL_ZW, 0.0f);

// /**
//  * Collective control power for inverse plant
//  * 	
//  * Control power for inverse plant
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */

// PARAM_DEFINE_FLOAT(HELI_COLL_CP, 0.0f);


// /**
//  * Collective offset
//  *
//  * Used for leveling the blades.
//  *
//  * @min 0.0
//  * @group Helicopter Attitude Control
//  */


// PARAM_DEFINE_FLOAT(HELI_COLL_OFF, 0.0f);


// 	