/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file digital_filter.h
 *
 * @brief Performs a Tustin transform on the digital filter parametrized by the 
 * 		  inputs. Source: https://en.wikipedia.org/wiki/Bilinear_transform
 * 
 */

#ifndef DIGITAL_FILTER_H_
#define DIGITAL_FILTER_H_

#include <px4_config.h>
#include <px4_defines.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <functional>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <platforms/px4_defines.h>
#include <heli_lib/c_signal/c_signal.h>


__BEGIN_DECLS

extern "C" __EXPORT float general_filter(float a0, float a1, float a2, float b0, float b1, float b2, 
					 					 float input, float in_prev1, float in_prev2, 
										 float out_prev1, float out_prev2, float dt);

extern "C" __EXPORT float filter_signal(float a0, float a1, float a2, float b0, float b1, float b2, 
					 					C_Signal input, C_Signal output, float dt);

extern "C" __EXPORT float derivative_signal(float K, C_Signal input, float dt);

extern "C" __EXPORT float integrate_signal(float K, C_Signal input, C_Signal output, float dt, float limit, bool isLimited);

extern "C" __EXPORT float lowpass_filter(float K, float cutoff_w, C_Signal input, C_Signal output, float dt);
__END_DECLS

#endif /* DIGITAL_FILTER_H_ */
