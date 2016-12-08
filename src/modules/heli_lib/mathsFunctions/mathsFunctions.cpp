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

#include "mathsFunctions.h"

float wrapPi(float angle)
{
	float angleOut;
	angleOut = (angle - M_PI)%(2 * M_PI) - M_PI;
	return angleOut;
} 


