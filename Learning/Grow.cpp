/* ---------------------------------------------------
	FILE:     Grow.cpp
	AUTHOR:   Josh Bongard
			  modified by Andreas and Daniel
	DATE:     14.11.2001
	FUNCTION: This file contains the main function for
	          the artificial biped with a neural net
			  exported from Nnetview and rebuilt.
	Version:  Version 1.1
 -------------------------------------------------- */

#include "Stdafx.h"
#include "stdio.h"
#include "stdlib.h"
#include <direct.h> //rf
#include "McdDtBridge.h"
#include "Mdt.h"
#include "MeViewer.h"
#include "MeMath.h"
#include <dinput.h>
#include "Agent.h"
#include "Constants.h"
#include "MathWorld.h"
#include "SimParams.h"
#include "Neuralnet.h"
#include "Learning.h"

extern struct	MeMemoryAPI MeMemoryAPIMalloc;
struct			MeMemoryOptions opts;

RRender			*rc;
MdtWorldID      world;
McdSpaceID      space;
McdDtBridgeID   cdHandler;
void			*mem;
const char	    **args;
SimParams		*simParams; // A collection of simulation parameters
MathWorld		*mathWorld; // The MathEngine virtual environment
Agent			*testAgent; // The simulated agent.
NeuralNet		*neuralNet; // the neural net
Learning		*learning;	// learning environment for testAgent
int				argNumber;
int				lfootcontact, rfootcontact, waistcontact; 
int				leftFootMaterial, rightFootMaterial, floorMaterial, waistMaterial;
int				q;			//variable for incrementing the input data file name

void cleanup(void)
{
	if ( testAgent != NULL )
		delete testAgent;
	delete learning;
	delete neuralNet;
	delete simParams;
}

void tick(RRender *rc)
{
	float *nvalues;
	float *svalues;
	
	RStopTimer(kRRender);
	// This function updates the virtual environment at each
	// tick of the clock. It is also the exit point for the
	// entire application. On exit, tick calls cleanup (see above),
	// and then stops the application.

	//Calculation of the new activation of the net (one timestep)
	//Motorneuronactivity of the neuralnet from exportfile will now drive the biped
	nvalues = new float[MOTORCOUNT];
	neuralNet->NetCalculate();
	neuralNet->NtfValueNewtoOld();
	neuralNet->NtfMotorWrite(nvalues);

	for (int i = 0; i < MOTORCOUNT; ++i)
	{ 
		if ((nvalues[i] == 0) && (nvalues[i+1] == 0))	
			testAgent->TurnOffMotor(i/2);
		else 
		{	
			if ((i == 0) || (i == 6)) // knee
			{
				if ((nvalues[i] == 0.0) && (nvalues[i+1] == 0.0))
				{
					testAgent->mode[i/2] = 2;
					testAgent->TurnOffMotor(i/2);
				}
				else if ((nvalues[i] != 0.0) && (nvalues[i+1] != 0.0) && (-0.2 < nvalues[i] - nvalues[i+1]) && (nvalues[i] - nvalues[i+1] < 0.2))
				{
					testAgent->TurnOnMotor(i/2);
					testAgent->mode[i/2] = 3; 
					testAgent->SetJointMoveValue(i/2, 0.4*(nvalues[i]-nvalues[i+1]), 5*(nvalues[i] + nvalues[i+1])/2); //real value 0.5 
				}
				else
				{
					testAgent->TurnOnMotor(i/2);
					
					if (nvalues[i] - nvalues[i+1] > 0.0)
					{
						testAgent->mode[i/2] = 1;
						testAgent->SetJointMoveValue(i/2, 0.7*(nvalues[i] - nvalues[i+1]));
					}
					else
					{
						testAgent->mode[i/2] = 4;
						testAgent->SetJointMoveValue(i/2, (nvalues[i] - nvalues[i+1]), 5*(nvalues[i] + nvalues[i+1])/2); 
					}
				}
			}
			else if ((i == 2) || (i == 4)) // hip
			{
				if ((nvalues[i] == 0.0) && (nvalues[i+1] == 0.0))
				{
					testAgent->mode[i/2] = 2;
					testAgent->TurnOffMotor(i/2);
				}
				else if ((nvalues[i] != 0.0) && (nvalues[i+1] != 0.0) && (-0.2 < nvalues[i] - nvalues[i+1]) && (nvalues[i] - nvalues[i+1] < 0.2))
				{
					testAgent->TurnOnMotor(i/2);
					testAgent->mode[i/2] = 3; 
					testAgent->SetJointMoveValue(i/2, 0.7*(nvalues[i]-nvalues[i]+nvalues[i+1])/2); //real value 1.5 
				}
				else
				{
					testAgent->TurnOnMotor(i/2);
					testAgent->mode[i/2] = 1;
					testAgent->SetJointMoveValue(i/2, 14*(nvalues[i] - nvalues[i+1]));
				}
			}
			else if ((i == 8) || (i == 10)) // ankle
			{ 
				if ((nvalues[i] == 0.0) && (nvalues[i+1] == 0.0))
				{
					testAgent->mode[i/2] = 2;
					testAgent->TurnOffMotor(i/2);
				}
				else
				{ 
					testAgent->TurnOnMotor(i/2);
					testAgent->mode[i/2] = 3;
					testAgent->SetJointMoveValue(i/2, 0.7*(nvalues[i+1] - nvalues[i]) , 5*(nvalues[i] + nvalues[i+1])/2); // 5
				} 
			}
		}
		++i;
	}

	// Run Agent
	testAgent->Move(q);
	svalues = new float[OUTPUTCOUNT];

    // golgi tendon organ values
	for ( i = 0; i < 12; ++i) //ovalues[i] = 1.0;
	{
		svalues[i] = 0.0;
		svalues[i] = testAgent->GetGolgiValues(i);
	}
	//muscle spindle values
	for ( i = 0; i < 12; ++i) //ovalues[i] = 1.0; 
	{
		svalues[i+12] = 0.0;
		svalues[i+12] = testAgent->GetMuscleSpindleValues(i);
 	}
	// cutaneous sensor values
	for ( i = 0; i < 2; ++i) //ovalues[i] = 1.0;
	{
		svalues[i+24] = 0.0;
		svalues[i+24] = testAgent->GetCutaneousValues(i);
	}
	for (i = 0; i < 12; i++)
	{
		svalues[i+26] = 0.0;
		svalues [i+26]= testAgent->GetMuscleDynamicSpindleValues(i);
	}

	if(SENSOR_ACTIVATE)
		neuralNet->NtfSensorRead(svalues);

/////////////////////////////////////////////////////////////////////////////
//learning stuff/////////////////////////////////////////////////////////////
	if (learningActivated)
	{
		learning->activateLearning(q, nvalues);

		if (learning->isTimeToRecreate())
		{
			delete testAgent;
			delete mathWorld;
			mathWorld = new MathWorld;
			testAgent = new Agent;
			learning->configuringNewAgent(testAgent, svalues);
		}
	}
//end of learning stuff//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
	
	q++;
	
	delete []svalues;
	delete []nvalues;

	RStartTimer(kRDynamics);
	MdtWorldStep(world, STEP_SIZE);
	RStopTimer(kRDynamics);

	RStartTimer(kRCollision);
	McdPairHandlerUpdate();
	RStopTimer(kRCollision);

	RStartTimer(kRRender);	
}

// main entry of program//////////////////////////////////////////////////////
int main(int argc, const char **argv)
{
	argNumber = argc;
	args = argv;

	// Create the four major classes of the application.
	simParams = new SimParams;
	mathWorld = new MathWorld;
	testAgent = new Agent;

	//Create and initialize a neural net from an exportfile from Nnetview
	neuralNet = new NeuralNet("nnetexp.txt",1);
	neuralNet->NtfRead();
	
	//create and initialize learning environment
	learning = new Learning(testAgent, neuralNet);

	// Specify which function should be called when the application ends.
	atexit(cleanup);

	// specify the camera position in graphic environment
	rc->m_cameraOffset = 10;
	rc->m_cameraAngle = PI*3/2;
	rc->m_cameraElevation = 0.5;
	RUpdateCamera();

	// The application now enters the tick function; all subsequent 
	// actions are started from within the tick function.
	q=0;
	RRun(rc, tick);
	
	return 0;
}