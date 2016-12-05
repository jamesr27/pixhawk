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
 * Encodes a command model 
 */
#include <px4_config.h>

#include <systemlib/param/param.h>

/**
 * 
 * Roll attitude input gain
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_GAIN, 0.5f);

/**
 * 
 * Roll natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_NF, 11.0f);

/**
 * 
 * Roll damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_DR, 0.9f);

/**
 * 
 * Roll velocity gain
 *
 * @unit rad/s/ deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_GAINV, 1.0f);

/**
 * 
 * Roll velocity natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_NFV, 5.0f);

/**
 * 
 * Roll velocity damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_DRV, 0.9f);
/**
 * 
 * Roll washout lag
 *
 * @unit 
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_WASH, 0.2f);

/**
 * 
 * Pitch attitude input gain
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_GAIN, 0.5f);

/**
 * 
 * Pitch natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_NF, 11.0f);

/**
 * 
 * Pitch damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_DR, 0.9f);

/**
 * 
 * Pitch rate input gain
 *
 * @unit rad/s/ deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_GAINV, 1.0f);

/**
 * 
 * Pitch natural frequency
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_NFV, 18.0f);

/**
 * 
 * Pitch damping ratio
 *
 * @unit rad/stick deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_DRV, 0.9f);
/**
 * 
 * Pitch washout tc
 *
 * @unit 
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_WASH, 0.2f);

/**
 * 
 * Pitch rate input gain
 *
 * @unit rad/s/ deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_YAWR_GAIN, 5.0f);

/**
 * 
 * Pitch lag
 *
 * @unit 
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_YAWR_LAG, 0.05f);


/**
 * 
 * Collective rate input gain
 *
 * @unit rad/s/ deflection
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_COLL_GAIN, 1.0f);

/**
 * 
 * Collective lag
 *
 * @unit 
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_COLL_LAG, 0.1f);

/**
 * 
 * Collective lag
 *
 * @unit 
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLLV_CLIM, 0.0f);

/**
 * 
 * Collective lag
 *
 * @unit 
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCHV_CLIM, 0.0f);

/**
 * 
 * Collective lag
 *
 * @unit 
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLLX_CLIM, 0.0f);

/**
 * 
 * Collective lag
 *
 * @unit 
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCHX_CLIM, 0.0f);

/**
 * 
 * Ramp rate for main rotor
 *
 * @unit 
 * @min 0.0
 * @group Helicopter Position Control
 */
PARAM_DEFINE_FLOAT(HELI_ROTOR_RATE, 0.0f);


