/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file heading_kinematics.h
 *
 * @brief Functions for doing kinematics given a heading. 
 * 		  Also just testing out Pixhawk class decl functionality. 
 * 
 */

#ifndef HEADING_KINEMATICS_H_
#define HEADING_KINEMATICS_H_


#include <float.h>
#include <stdint.h>
#include <systemlib/systemlib.h>
#include <math.h>
#include <mathlib/mathlib.h>

__BEGIN_DECLS

extern "C" __EXPORT float wrapPi(float angle);


__END_DECLS

#endif /* HEADING_KINEMATICS_H_ */
