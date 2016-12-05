/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file digital_filter.cpp
 *
 * @brief Performs a Tustin transform on the digital filter parametrized by the 
 * 		  inputs. Source: https://en.wikipedia.org/wiki/Bilinear_transform
 * 
 */

#include "digital_filter.h"


float general_filter(float a0, float a1, float a2, float b0, float b1, float b2, 
					 float input, float in_prev1, float in_prev2, 
					 float out_prev1, float out_prev2, float dt) 
{

	float K = 2.0f/dt;
	float in0 = b0*powf(K, 2.0f) + b1*K + b2;
	float in1 = 2.0f*b2 - 2.0f*b0*powf(K, 2.0f);
	float in2 = b0*powf(K, 2.0f) - b1*K + b2;
	float out1 = 2.0f*a2 - 2.0f*a0*powf(K, 2.0f);
	float out2 = a0*powf(K, 2.0f) - a1*K + a2;
	float denom = a0*powf(K, 2.0f) + a1*K + a2;

	// printf("a0 %.3f, a1 %.3f, a2 %.3f, b0 %.3f, b1 %.3f, b2 %3f\n", (double) a0, (double) a1, (double) a2, (double) b0, (double) b1, (double) b2);
	// printf("K: %.3f, in0 %.3f, in1 %.3f, in2 %.3f, out1 %.3f, out2 %.3f, denom %.3f\n", (double) K, (double) in0, (double) in1, (double) in2, (double) in2, (double) out1, (double) out2, (double) denom);
	// printf("input.current: %.3f, input.previous: %.3f, output.previous: %.3f, output.previous2 %.3f\n", (double) input, (double) in_prev1, (double) in_prev2, (double) out_prev1, (double) out_prev2);
	return (in0 * input + in1 * in_prev1 + in2 * in_prev2 - out1 * out_prev1 - out2 * out_prev2)/denom;

}

//Takes a signal object instead of each element. 
//Be aware of weirdness surrounding the "output current". We actually don't have an output current yet, the output from the filter is going to be that soon (will bump everything)

float filter_signal(float a0, float a1, float a2, float b0, float b1, float b2, 
					 C_Signal input, C_Signal output, float dt) 
{
	return general_filter(a0, a1, a2, b0, b1, b2, input.current(), input.previous(), 
						input.previous2(), output.current(), output.previous(), dt);	
}

//Discrete time derivative based on bilinear approximation

float derivative_signal(float K, C_Signal input, float dt) 
{
	//return  general_filter(0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, input.current(), input.previous(),
	//						input.previous2(), output.current(), output.previous(), dt);
	return K * (input.current() - input.previous2())/(2.0f * dt);
	//K*(2.0f/dt) * (input.current() - input.previous()) - output.current();
	//return K*(input.current() - input.previous())/dt;
}

//Integrates a signal. 

float integrate_signal(float K, C_Signal input, C_Signal output, float dt, float limit, bool isLimited) 
{
	float integrated_val = general_filter(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, K, input.current(), input.previous(),
						 input.previous2(), output.current(), output.previous(), dt); //Output current is really the previous output, the output of the filter will become the new output current. 
	if(!isLimited) { //Numerical hack. 
	
		return integrated_val; //To just return the unlimited thing. 
	
	} else {
		
		if(fabsf(integrated_val) < limit) {
			
			return integrated_val;
		
		} else { 
			if(integrated_val < 0)
				
				return -limit; //With a limit on it.
			
			else return limit;
		}
	}
}


float lowpass_filter(float K, float cutoff_w, C_Signal input, C_Signal output, float dt) 
{

	return general_filter(0.0f, 1.0f/cutoff_w, 1.0f, 0.0f, 0.0f, K, input.current(), input.previous(),
						 input.previous2(), output.current(), output.previous(), dt);
}

