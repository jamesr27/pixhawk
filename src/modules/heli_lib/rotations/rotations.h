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

extern "C" __EXPORT math::Vector<2> in_plane_rotation(float angle, math::Vector<2> rot_vec);

extern "C" __EXPORT math::Vector<3> euler_321_rotation(float phi, float theta, float psi, math::Vector<3> rot_vec);
extern "C" __EXPORT math::Vector<4> euler_321_rotation4d(float phi, float theta, float psi, math::Vector<4> rot_vec);

extern "C" __EXPORT math::Vector<3> strapdown_rotation(float phi, float theta, math::Vector<3> rot_vec); //Note singularity at pi/2

extern "C" __EXPORT math::Matrix<3,3> L1_rotation(float angle);

extern "C" __EXPORT math::Matrix<3,3> L2_rotation(float angle);

extern "C" __EXPORT math::Matrix<3,3> L3_rotation(float angle);

__END_DECLS

#endif /* HEADING_KINEMATICS_H_ */
