/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file heli_path_planner_params.c
 *
 * @brief Helicopter autonomous controller parameters
 *
 * Parameters for path planning.  
 */
#include <px4_config.h>

#include <systemlib/param/param.h>

/**
 * 
 * Gamma target
 *
 * @unit radians
 */

PARAM_DEFINE_FLOAT(PATH_GAMMA_T, 0.0f);

/**
 * 
 * Elevation target
 *
 * @unit radians
 */

PARAM_DEFINE_FLOAT(PATH_ELEV_T, 0.0f);


/**
 * 
 * Take off target
 *
 * @unit radians
 */

PARAM_DEFINE_FLOAT(PATH_TO_ELEV, 0.0f);

/**
 * 
 * Take off lead distance
 *
 * @unit radians
 */

PARAM_DEFINE_FLOAT(PATH_TO_LEAD, 0.0f);


/**
 * 
 * Angle rate transition
 *
 * @unit radians
 */

PARAM_DEFINE_FLOAT(PATH_RATE_ANGLE, 0.0f);



/**
 * 
 * Position rate transition
 *
 * @unit distance
 */

PARAM_DEFINE_FLOAT(PATH_RATE_POS, 0.0f);

