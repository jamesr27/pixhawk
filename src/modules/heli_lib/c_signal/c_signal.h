/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file signal.h
 *
 * @brief Contains a signal with its histories. 
 * 
 */

#ifndef C_SIGNAL_H_
#define C_SIGNAL_H_

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


class __EXPORT C_Signal
{
	public: 

		//Constructor. Initializes to 0. 
		C_Signal();

		// Deconstructor. Clears everything to 0. 
		~C_Signal() {}



		//Does what function names say. 
		float previous();

		float previous2();

		//Gets the current signal state
		float current();
		//Assigns the current signal state alone without bumping. 
		void current(float signal_value);
		
		// Assigns the newest value to 0 and pushes the rest down. 
		void bump_current(float signal_value);

		//Modify arbitrary points if so desired. 
		void modify_history(int index, float signal_value);

		void set_all(float signal_value);
		//Reinitializes the vector to all 0s. 
		void zero();

		void print();

	private: 
 
		math::Vector<3> _signal;

};


#endif /* C_SIGNAL_H_ */ 
