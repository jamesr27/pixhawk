/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file heading_kinematics.cpp
 *
 * @brief Functions for doing kinematics given a heading. 
 * 		 Performs an inplane rotation 2D rotation give an angle 
 * 		 and an angle to rotate. 
 * 
 */

#include "rotations.h"

math::Vector<2> 
in_plane_rotation(float angle, math::Vector<2> rot_vec)
{
	math::Matrix<2,2> rotation;
	rotation(0, 0) = cosf(angle);
	rotation(0, 1) = sinf(angle);
	rotation(1, 0) = -rotation(0, 1);
	rotation(1, 1) = rotation(0, 0);
	return rotation * rot_vec;
} 

math::Vector<3> 
strapdown_rotation(float phi, float theta, math::Vector<3> rot_vec)
{

	math::Matrix<3,3> strapdown;
	strapdown.identity();
	float c_theta = cosf(theta);
	//float s_theta = sinf(theta);
	float t_theta = tanf(theta);

	float c_phi = cosf(phi);
	float s_phi = sinf(phi);
	
	strapdown(0, 1) = s_phi * t_theta;
	strapdown(0, 2) = c_phi * t_theta;
	strapdown(1, 1) = c_phi;
	strapdown(1, 2) = -s_phi;

	if(c_theta < .01f && c_theta > 0.0f) 
		c_theta = .01f;
	else if(c_theta > -.01f && c_theta < 0.0f)
		c_theta = -.01f;

	strapdown(2, 1) = s_phi/c_theta;
	strapdown(2, 2) = c_phi/c_theta;
 	return strapdown * rot_vec;
}

math::Vector<3> 
euler_321_rotation(float phi, float theta, float psi, math::Vector<3> rot_vec)
{

	return (L1_rotation(phi)*L2_rotation(theta)*L3_rotation(psi))*rot_vec;
	
}

math::Vector<4> //For accomodating the yaw too
euler_321_rotation4d(float phi, float theta, float psi, math::Vector<4> rot_vec)
{
	math::Vector<3> new_vec(rot_vec(0), rot_vec(1), rot_vec(2));
	new_vec = (L1_rotation(phi)*L2_rotation(theta)*L3_rotation(psi))*new_vec;
	return math::Vector<4> (new_vec(0), new_vec(1), new_vec(2), rot_vec(3));
	
}

math::Matrix<3,3>
L1_rotation(float angle)
{
	math::Matrix<3,3> L1;
	L1.identity();
	L1(1, 1) = cosf(angle);
	L1(2, 2) = cosf(angle);
	L1(1, 2) = sinf(angle);
	L1(2, 1) = -sinf(angle);
	return L1;
 }

math::Matrix<3,3>
L2_rotation(float angle)
{
	math::Matrix<3,3> L2;
	L2.identity();
	L2(0, 0) = cosf(angle);
	L2(2, 2) = cosf(angle);
	L2(0, 2) = -sinf(angle);
	L2(2, 0) = sinf(angle);
	return L2;

}

math::Matrix<3,3>
L3_rotation(float angle)
{
	math::Matrix<3,3> L3;
	L3.identity();
	L3(0, 0) = cosf(angle);
	L3(1, 1) = cosf(angle);
	L3(0, 1) = sinf(angle);
	L3(1, 0) = -sinf(angle);
	return L3;
}


