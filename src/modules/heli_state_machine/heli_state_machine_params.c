/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_tracking_control_params.c
 *
 * @brief Params for state machine
 *
 * Parameters for safety cutoffs, etc.. 
 */

#include <px4_config.h>

#include <systemlib/param/param.h>
 
PARAM_DEFINE_INT32(STATE_RC_CONTROL, 0.0f);
 
PARAM_DEFINE_FLOAT(STATE_LIDAR_CUT, 0.0f);