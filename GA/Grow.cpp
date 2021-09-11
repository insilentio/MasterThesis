/* ---------------------------------------------------
   FILE:     Grow.cpp
	AUTHOR:   Josh Bongard
			  modified by Andreas and Daniel
	DATE:     30.08.2001
	FUNCTION: This file contains the main function for
	          the artificial biped with a neural net
			  exported from Nnetview and rebuilt.
	Version:  Version 1
 -------------------------------------------------- */

#include "stdafx.h"
#include "stdio.h"
#include "stdlib.h"
#include <direct.h> //rf
//#include <string.h>

#include "McdDtBridge.h"
#include "Mdt.h"
#include "MeViewer.h"
#include "MeMath.h"


#include "agent.h"
#include "constants.h"
#include "mathWorld.h"
#include "simParams.h"
#include "channel.h"
#include "growGA.h"
#include "neuralnet.h"
#include <dinput.h>

extern struct          MeMemoryAPI MeMemoryAPIMalloc;
struct MeMemoryOptions opts;

RRender			*rc;
MdtWorldID      world;
McdSpaceID      space;
McdDtBridgeID   cdHandler;
void			*mem;

int			    argNumber;
const char	    **args;

SIM_PARAMS		*simParams; // A collection of simulation parameters
MATH_WORLD		*mathWorld; // The MathEngine virtual environment
AGENT			*testAgent; // The simulated agent.
GROW_GA			*growGA;
NeuralNet		*neuralNet;
Channel			*ChannelRetour;

extern const int MOTORCOUNT;
extern const int JOINT_ANGLES_COUNT;
extern int		 TICK_START;
extern int		 TICK_BIS_REFLEX_START;
extern int		 GA_WEIGHTSCOUNT;	//this constant does still have to be countet!!
extern int		 ANGLES_MN_WRITE;
extern int		 REFLEX_ACTIVATE;
extern int		 SENSOR_ACTIVATE;

int lfootcontact; 
int rfootcontact;
int waistcontact;
int LeftFootMaterial, RightFootMaterial, floorMaterial, waistMaterial;
//char buffer[2];		//variable for converting int to string; not necessary for switch-case
int q, run;					//variable forincrementing the input data file name
int nextStep;


void cleanup(void)
{
	// This function removes the agent, the virtual environment,
	// The genetic algorithm, and the simulation parameters from
	// memory at the end of the application.

	if ( testAgent != NULL )
		delete testAgent;
	
	// Close and Delete Neuralnet
	
	delete neuralNet;
	delete ChannelRetour;

	
	delete simParams;
}

void tick(RRender *rc)
{
	float *nvalues;
	float *svalues;
	float jointAngles[JOINT_ANGLES_COUNT];
	float channelValues[MOTORCOUNT+JOINT_ANGLES_COUNT];
	char directory[16];
	
	nextStep = 0;

	RStopTimer(kRRender);
	
	// This function updates the virtual environment at each
	// tick of the clock. It is also the exit point for the
	// entire application, when the genetic algorithm has been
	// completely evaluated. On exit, tick calls cleanup (see above),
	// and then stops the application.

	if ( !simParams->evalOver ) {

	//Calculating of the new activation of the net (one timestep)
	//printf("nach NetCalculate()\n");
	//Motorneuronactivity of the neuralnet from exportfile will now drive the biped
	

	nvalues = new float[MOTORCOUNT];
	neuralNet->NetCalculate();
	neuralNet->NtfValueNewtoOld();
	neuralNet->NtfMotorWrite(nvalues);

	for (int i = 0; i < MOTORCOUNT; ++i)
	{ 
		if(q>17)
		{
			testAgent->MuscleactivityCalculate(nvalues[i]);
		}
		
		if ((nvalues[i] == 0) && (nvalues[i+1] == 0))	
			testAgent->TurnOffMotor(i/2);
		else 
		{	
			//knee
			if ((i == 0) || (i == 6)) {
				if ((nvalues[i] == 0.0) && (nvalues[i+1] == 0.0)) {
					testAgent->mode[i/2] = 2;
					testAgent->TurnOffMotor(i/2);
				}
				else if ((nvalues[i] != 0.0) && (nvalues[i+1] != 0.0) && (-0.2 < nvalues[i] - nvalues[i+1]) && (nvalues[i] - nvalues[i+1] < 0.2)) {
					testAgent->TurnOnMotor(i/2);
					testAgent->mode[i/2] = 3; 
					testAgent->SetJointMoveValue(i/2, 0.4*(nvalues[i]-nvalues[i+1]), 5*(nvalues[i] + nvalues[i+1])/2); //real value 0.5 
				}
				else {
					testAgent->TurnOnMotor(i/2);
					if (nvalues[i] - nvalues[i+1] > 0.0) {
					testAgent->mode[i/2] = 1;
					testAgent->SetJointMoveValue(i/2, 0.7*(nvalues[i] - nvalues[i+1]));
					}
					else { testAgent->mode[i/2] = 4;
					testAgent->SetJointMoveValue(i/2, (nvalues[i] - nvalues[i+1]), 5*(nvalues[i] + nvalues[i+1])/2); 
					}
					}
			}
			//hip
			else if ((i == 2) || (i == 4)) {
				if ((nvalues[i] == 0.0) && (nvalues[i+1] == 0.0)) {
					testAgent->mode[i/2] = 2;
					testAgent->TurnOffMotor(i/2); }
				else if ((nvalues[i] != 0.0) && (nvalues[i+1] != 0.0) && (-0.2 < nvalues[i] - nvalues[i+1]) && (nvalues[i] - nvalues[i+1] < 0.2)) {
					testAgent->TurnOnMotor(i/2);
					testAgent->mode[i/2] = 3; 
					testAgent->SetJointMoveValue(i/2, 0.7*(nvalues[i]-nvalues[i]+nvalues[i+1])/2); //real value 1.5 
				}
				else {
					testAgent->TurnOnMotor(i/2);
					testAgent->mode[i/2] = 1;
					testAgent->SetJointMoveValue(i/2, 14.0*(nvalues[i] - nvalues[i+1]));
					}
			}

			//ankle
			else if ((i == 8) || (i == 10)) { 
				if ((nvalues[i] == 0.0) && (nvalues[i+1] == 0.0)) {
					testAgent->mode[i/2] = 2;
					testAgent->TurnOffMotor(i/2);
				}
				else { 
					testAgent->TurnOnMotor(i/2);
					testAgent->mode[i/2] = 3;
					testAgent->SetJointMoveValue(i/2, 0.7*(nvalues[i+1] - nvalues[i]) , 12*(nvalues[i] + nvalues[i+1])/2); // 5
				} 

			} // else if 
			

		} // else
		 ++i;
	}


	// Run Agent
	testAgent->Move(q);
	svalues = new float[OUTPUTCOUNT];
    // golgi tendon organ values
	for ( i = 0; i < 12; ++i) 
	{
		svalues[i] = 0.0;
		svalues[i] = testAgent->GetGolgiValues(i);
	}
 //muscle spindle values
   for ( i = 0; i < 12; ++i) 
	{
		svalues[i+12] = 0.0;
		svalues[i+12] = testAgent->GetMuscleSpindleValues(i);
	}
	
	// cutaneous sensor values
	for ( i = 0; i < 2; ++i) 
	{
		svalues[i+24] = 0.0;
		svalues[i+24] = testAgent->GetCutaneousValues(i);
		 
	}
	for (i=0; i<12;i++)
	{
		svalues[i+26] = 0.0;
		svalues [i+26]= testAgent->GetMuscleDynamicSpindleValues(i);

	}

	if(REFLEX_ACTIVATE)
		neuralNet->NtfReflexActivate(svalues,12,q);

	if(SENSOR_ACTIVATE)
		neuralNet->NtfSensorRead(svalues);
	
//-------------------------------------------------------------------------------------
// to export angles and mn-acitvity into a txt-file
//-------------------------------------------------------------------------------------
	if((ANGLES_MN_WRITE)&&(run==1))
	{
		for(int k = 0; k < JOINT_ANGLES_COUNT/2; k++)
		{	
			jointAngles[k] = testAgent->getJointAngle(k);

			if ((k==0)||(k==3)||(k==6)||(k==9))
				jointAngles[k]= jointAngles[k]- 0.572801;
			else if((k==1)||(k==7))
				jointAngles[k]= (jointAngles[k] + 0.0579246)*-1;
			else if((k==2)||(k==8))
				jointAngles[k]= (jointAngles[k] -0.057459);//*-1;
			else if((k==4)||(k==5)||(k==10)||(k==11))
				jointAngles[k]= (jointAngles[k]- 0.0006) *-1;
		}
		for(k = 0; k < MOTORCOUNT+JOINT_ANGLES_COUNT; k++)
		{
			if(k < MOTORCOUNT)
				channelValues[k] = nvalues[k];
			else
				channelValues[k] = jointAngles[k-MOTORCOUNT];
		}


		if (q>0 && q<2000)// ***********************schalter für export daten
		{

			testAgent->WritetoFile(q, channelValues);

		}
	}
//-----------------------------------------------------------------------------------

	delete []svalues;
	delete []nvalues;
//	delete []directory;
	q++;
/*	if (q==TICK_START)
		cin>>nextStep;*/
/*	cout<<q<<endl;
	if ((q > TICK_BIS_REFLEX_START) && (q < TICK_BIS_REFLEX_START+200)) 
		cin>>nextStep;*/
	}

	else 
	{

		growGA->SetFitness( testAgent->GetFitness() );
	
		growGA->DisplayCurrentGenome();

		if ( !FUNCTION_AS_READER )
			growGA->NextGenome();

		if ( !growGA->finished )
		{
	 		delete testAgent;

			if ( neuralNet != NULL )
				delete neuralNet;

			delete mathWorld;

			mathWorld = new MATH_WORLD;

			neuralNet = new NeuralNet("nnetexp.txt",1);
			neuralNet->NtfRead();	//reads in the neural net

			neuralNet->NtfWriteWeightsToGenome();	
			
			const float *nums = growGA->GetCurrentGenome()->genes;

			testAgent = new AGENT;

			neuralNet->NtfGAChangeWeights(nums); //Function to change the weights by the GA
			
			simParams->evalOver = false;

			q=0;
			run++;
		}
		else
			exit(0); 
	}

	RStartTimer(kRDynamics);
	MdtWorldStep(world, STEP_SIZE);
	RStopTimer(kRDynamics);

	RStartTimer(kRCollision);
	McdPairHandlerUpdate();
	RStopTimer(kRCollision);

	RStartTimer(kRRender);	
	//Sleep(0);
}

int main(int argc, const char **argv)
{
	argNumber = argc;
	args = argv;

	// Create the four major classes of the application.
	simParams = new SIM_PARAMS;

	simParams->runNumber = atoi(argv[2]);

	srand(simParams->runNumber);

	mathWorld = new MATH_WORLD;

	testAgent = new AGENT;

	//Create and Initialize Communication Channels with Nnetview

	//j=1; //Variable für das Durchzählen zur Einlesung der EMG-Textfiles
    //ChannelInp = new Channel("dtfmi.txt", 1);
    //ChannelOut = new Channel("dtfmo.txt", 0);
  	//ChannelRetour= new Channel("dtfmretour.txt", 0);

	// Specify which function should be called when
	// the application ends.
	atexit(cleanup);

	rc->m_cameraOffset = 12;
	rc->m_cameraAngle = PI;
	rc->m_cameraElevation = 0.38;
	RUpdateCamera();

	//Create and initialize a neural net from an exportfile from Nnetview
	neuralNet = new NeuralNet("nnetexp.txt",1);
	neuralNet->NtfRead();

	if (FUNCTION_AS_RESTARTER)
		neuralNet->NtfGAWeightstoNeuralNet();

	neuralNet->NtfWriteWeightsToGenome();
	growGA	  = new GROW_GA;

	q=0;
	run=1;
	cin>>nextStep;
	
	// The application now enters the tick function; all
	// subsequent actions are started from within the
	// tick function.
	
	RRun(rc, tick);

	return 0;
}