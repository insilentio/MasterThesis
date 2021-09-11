/* ---------------------------------------------------
   FILE:     simParams.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 5, 2000
	FUNCTION: This class contains all miscellaneous
				 data and functions for this simulation.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _SIM_PARAMS_CPP
#define _SIM_PARAMS_CPP

#include "simParams.h"
#include "stdlib.h"
#include "string.h"
#include "agent.h"

extern int GA_WEIGHTSCOUNT;

SIM_PARAMS::SIM_PARAMS(void) {

	printf("Creating simulation parameters.\n");

	rendererCreated = false;
	evalOver = false;
	wrvalues = new float[GA_WEIGHTSCOUNT];
}

SIM_PARAMS::~SIM_PARAMS(void) {

	printf("Destroying simulation parameters.\n");
}

int	SIM_PARAMS::FlipCoin(void) {

	int randNum = rand();
	float zeroToOne = ((float)rand()) / RAND_MAX;

	return ( zeroToOne < 0.5 );
}

float SIM_PARAMS::Rand(float min, float max) {

	int randNum = rand();
	float zeroToOne = ((float)rand()) / RAND_MAX;
	float returnVal;

	returnVal = (zeroToOne * (max-min)) + min;
	return returnVal;
}

int SIM_PARAMS::RandInt(int min, int max) {

	return( (rand() % (max-min+1)) + min );
}

float SIM_PARAMS::Scale(float value,float lowerLimit, float upperLimit) {

	if ( value < 0.0 )
		return( -(lowerLimit*value) );
	else
		return( upperLimit*value );
}

void SIM_PARAMS::Write(float *weights, int i)
{
	wrvalues[i] = weights[i];
//	cout<<wrvalues[i]<<"Kopie und Orginal"<<weights[i]<<"Position"<<i<<endl;
}

float SIM_PARAMS::GetWeights(int i)
{
	return wrvalues[i];
}
int SIM_PARAMS::CheckWeightRange(int i)
{
	if ((wrvalues[i]<0)&&(wrvalues[i]>=-1))
		return -1;
	else if((wrvalues[i]>0)&&(wrvalues[i]<=1))
		return 1;
	else if(wrvalues[i]>1.0)
		return 10;
	else if (wrvalues[i]<-1)
		return -10;
	else if (wrvalues[i]==0)
		return 0;
}

#endif