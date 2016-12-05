/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file signal.h
 *
 * @brief Create pulses. For input testing. 
 * 
 */

#ifndef PULSE_HPP_
#define PULSE_HPP_

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


class __EXPORT Pulse
{
	public: 

		//Constructor. Initializes to 0. 
		Pulse() : 
			_amplitude(0),
			_time_elapsed(0),
		{

		}

		virtual	~Pulse() {};

		float _amplitude;
		float _time_elapsed;
		uint64_t _initial_time; 
		float _current_state;
};


#endif /* PULSE_HPP_ */ 
