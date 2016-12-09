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
	angleOut = (float)fmod(((double)angle + M_PI),(2.0 * M_PI));

	if (angleOut < 0)
	{
		angleOut += 2.0f * (float)M_PI;
	}

	angleOut = angleOut - (float)M_PI;

	return angleOut;
}

