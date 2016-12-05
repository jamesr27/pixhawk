/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file signal.cpp
 *
 * @brief Implementation. 
 * 
 */

#include "c_signal.h"

//Trivial constructor. 
C_Signal::C_Signal()
{ 
	_signal.zero();
}

//Returns the previous state. 
float C_Signal::previous()
{
	return _signal(1);
}

float C_Signal::previous2()
{
	return _signal(2);
}

float C_Signal::current()
{

	return _signal(0);
}

void C_Signal::current(float signal_value)
{
	_signal(0) = signal_value;
}

void C_Signal::bump_current(float signal_value)
{
	_signal(2) = _signal(1);
	_signal(1) = _signal(0);
	_signal(0) = signal_value;
}

void C_Signal::modify_history(int index, float signal_value) 
{
	if(index >= 0 && index < 3)
		_signal(index) = signal_value;
}

void C_Signal::zero()
{
	_signal.zero();
}

void C_Signal::set_all(float signal_value)
{
	for(int i = 0; i < 3; i++)
	{
		_signal(i) = signal_value;
	}
}


void C_Signal::print() {
	_signal.print();
}

