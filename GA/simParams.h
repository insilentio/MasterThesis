/* ---------------------------------------------------
   FILE:     simParams.h
	AUTHOR:   Josh Bongard
	DATE:     October 5, 2000
	FUNCTION: This class contains all miscellaneous
				 data and functions for this simulation.
 -------------------------------------------------- */

#include "stdafx.h"
#include "fstream.h"

#ifndef _SIM_PARAMS_H
#define _SIM_PARAMS_H

class SIM_PARAMS {

public:
	int		rendererCreated;
	ifstream	inBestAgentFile;
	int		runNumber;
	int      evalOver;
	float	*wrvalues;

public:
	SIM_PARAMS(void);
	~SIM_PARAMS(void);
	int   FlipCoin(void);
	float Rand(float min, float max);
	int   RandInt(int min, int max);
	float Scale(float value, float lowerLimit, float upperLimit);
	void Write(float *weights,int i);
	float GetWeights(int i);
	int CheckWeightRange(int i);
};

#endif