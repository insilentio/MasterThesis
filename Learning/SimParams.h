/* ---------------------------------------------------
   FILE:     SimParams.h
	AUTHOR:   Josh Bongard
	DATE:     October 5, 2000
	FUNCTION: This class contains all miscellaneous
				 data and functions for this simulation.
 -------------------------------------------------- */

#include "Stdafx.h"
#include "fstream.h"

#ifndef _SimParams_H
#define _SimParams_H

class SimParams {

public:
	int		rendererCreated;
	ifstream	inBestAgentFile;
	int      evalOver;

public:
	SimParams(void);
	~SimParams(void);
	int   FlipCoin(void);
	float Rand(float min, float max);
	int   RandInt(int min, int max);
	float Scale(float value, float lowerLimit, float upperLimit);
};

#endif