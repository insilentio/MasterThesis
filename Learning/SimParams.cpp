/* ---------------------------------------------------
   FILE:     SimParams.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 5, 2000
	FUNCTION: This class contains all miscellaneous
				 data and functions for this simulation.
 -------------------------------------------------- */

#include "Stdafx.h"

#ifndef _SimParams_CPP
#define _SimParams_CPP

#include "SimParams.h"
#include "stdlib.h"
#include "string.h"
#include "Agent.h"

SimParams::SimParams(void) {

	printf("Creating simulation parameters.\n");

	rendererCreated = false;
	evalOver = false;
}

SimParams::~SimParams(void) {

	printf("Destroying simulation parameters.\n");
}

int	SimParams::FlipCoin(void) {

	int randNum = rand();
	float zeroToOne = ((float)rand()) / RAND_MAX;

	return ( zeroToOne < 0.5 );
}

float SimParams::Rand(float min, float max) {

	int randNum = rand();
	float zeroToOne = ((float)rand()) / RAND_MAX;
	float returnVal;

	returnVal = (zeroToOne * (max-min)) + min;
	return returnVal;
}

int SimParams::RandInt(int min, int max) {

	return( (rand() % (max-min+1)) + min );
}

float SimParams::Scale(float value, float lowerLimit, float upperLimit) {

	if ( value < 0.0 )
		return( -(lowerLimit*value) );
	else
		return( upperLimit*value );
}

#endif