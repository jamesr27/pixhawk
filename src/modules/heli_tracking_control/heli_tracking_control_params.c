/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_tracking_control_params.c
 *
 * @brief Helicopter autonomous contYer parameters
 *
 * Encodes a command model 
 */
#include <px4_config.h>

#include <systemlib/param/param.h>

/**
 * 
 * y attitude input gain
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_ROLL_GAIN, 0.0f);

/**
 * 
 * ROLL natural frequencroll
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_ROLL_NF, 0.0f);

/**
 * 
 * ROLL damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_ROLL_DR, 0.0f);

/**
 * 
 * ROLL velocitroll gain
 *
 * @unit rad/s/ deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_ROLL_V, 0.0f);

/**
 * 
 * ROLL velocitroll natural frequencroll
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_ROLL_P, 0.0f);

/**
 * 
 * ROLL velocitroll damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_ROLL_I, 0.0f);

/**
 * 
 * PITCH attitude input gain
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_PITCH_GAIN, 0.0f);

/**
 * 
 * PITCH natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_PITCH_NF, 0.0f);

/**
 * 
 * PITCH damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_PITCH_DR, 0.0f);

/**
 * 
 * PITCH velocity gain
 *
 * @unit rad/s/ deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_PITCH_V, 0.0f);

/**
 * 
 * PITCH velocity natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_PITCH_P, 0.0f);

/**
 * 
 * PITCH velocity damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_PITCH_I, 0.0f);

/**
 * 
 * yaw attitude input gain
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_YAW_GAIN, 0.0f);

/**
 * 
 * yaw natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_YAW_NF, 0.0f);

/**
 * 
 * yaw damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_YAW_DR, 0.0f);

/**
 * 
 * yaw velocity gain
 *
 * @unit rad/s/ deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_YAW_V, 0.0f);

/**
 * 
 * yaw velocity natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_YAW_P, 0.0f);

/**
 * 
 * yaw velocity damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_YAW_I, 0.0f);

/**
 * 
 * yaw attitude input gain
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_COLL_GAIN, 0.0f);

/**
 * 
 * coll natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_COLL_NF, 0.0f);

/**
 * 
 * coll damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_COLL_DR, 0.0f);

/**
 * 
 * coll velocity gain
 *
 * @unit rad/s/ deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_COLL_V, 0.0f);

/**
 * 
 * coll velocity natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_COLL_P, 0.0f);

/**
 * 
 * coll velocity damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(TRACK_COLL_I, 0.0f);

