//////////////////////////////////////////////////////////////////////
//	FILE:		Learning.cpp
//	AUTHOR:		Daniel Baumgartner
//	DATE:		October 8, 2001
//	FUNCTION:	Implementation of class Learning
//				changing weights of selected connections accordingly to
//				reward/penalty and value functions which are calculated
//				also inside this class
//	VERSION:	1.01
//////////////////////////////////////////////////////////////////////

#include "Stdafx.h"
#include "Learning.h"
#include "NeuralNet.h"

#if !defined(AFX_LEARNING_CPP__F3570235_5D71_4637_852B_9DB67A9D1301__INCLUDED_)
#define AFX_LEARNING_CPP__F3570235_5D71_4637_852B_9DB67A9D1301__INCLUDED_

// extern constants (defined in constants.h)
extern double	REWARD_AMOUNT;
extern double	LEARNING_RATE;
extern double	FORGETTING_RATE;
extern double	VALUE_RATE;
extern int		LEARNING_STEPS;
extern int		REWARD_VERSION;
extern int		STEPCYCLES;
extern int		WAITCYCLES;
extern int		JOINTS;
extern int		CONNECTIONCOUNT;
extern int		UNITCOUNT;
extern int		RESTRAINT;
extern int		JOINT_ANGLES_COUNT;
extern int		MOTORCOUNT;
extern short	RIGHT_HIP_ID;
extern short	LEFT_HIP_ID;
extern short	RIGHT_ANKLE_ID;
extern short	RIGHT_KNEE_ID;
extern float	POS_DEVIATION;
extern short	AVERAGE;
extern short	SELECTION;

// the following ones are needed for adjusting after agent and math world have been recreated
extern float	AGENT_CHANGE;
extern float	DEFAULT_RADIUS;
extern float	DEF_CONNECTOR_MASS;
extern float	DEF_LL_LENGTH;
extern float	DEF_UL_LENGTH;
extern float	DEF_LL_MASS;
extern float	DEF_UL_MASS;
extern float	DEF_LL_WIDTH;
extern float	DEF_UL_WIDTH;
extern float	DEF_FOOT_LENGTH;
extern float	DEF_FOOT_MASS;
extern float	DEF_FOOT_WIDTH;
extern float	DEF_FOOT_THICKNESS;
extern float	BODY_WIDTH;
extern float	DEF_WAIST_MASS;
extern float	DEF_WAIST_WIDTH;
extern float	TARGET_SIZE;
extern float	HEIGHT_OFFSET;
extern int		SENSOR_ACTIVATE;

// extern flags (defined in constants.h)
extern bool		valueInclude;
extern bool		debug;
extern bool		letAgentGrow;
extern bool		agentDetPos;
extern bool		includeSensory;
extern bool		updateWithGAWeights;
extern bool		stepcyclePrintout;
extern bool		mnOutput;

extern char		*gaweights;


//////////////////////////////////////////////////////////////////////////////////
// Construction
//////////////////////////////////////////////////////////////////////////////////

Learning::Learning(Agent *testAgent, NeuralNet *neuralNet)
{
	cout << "Creating a learning environment" << endl << endl << "Initializing..." << endl;

	allocateAgent(testAgent);
	setJointIDs();

	// initializing variables
	net = neuralNet;

	halfStepCycle = 0;
	newDistance = 0.0;
	rearmostPosRight = 0.0;
	oldFoot = 0.0;
	newFoot = 0.0;
	kneePos = 0.0;
	newMovementDirection = 0;
	oldMovementDirection = 0;
	halfStepCycle = 0;
	securityCheck = 0;
	fillingAngle = 0;
	learningStarted = 0;
	agentCount = 1;
	oldCount = 1;
	s = 0;
	stepcounter = 0;
	fillRuns = 0;
	learnRuns = 0;
	temp = 0.0;
	offset = 0;
	sourceActivity = 0.0;
	targetActivity = 0.0;
	newStrength = 0.0;
	oldStrength = 0.0;
	outputCounter = 0;

	firstTime = true;
	isTime = false;
	overZero = false;
	belowZero = false;
	switched = false;
	fillingMode = true;
	isRunning = false;
	smaller = false;
	learning10finished = false;
	learning100finished = false;
	learning1000finished = false;
	updated = false;

	jointAngles = new double[JOINT_ANGLES_COUNT/2];
	outputValues = new double[MOTORCOUNT + JOINT_ANGLES_COUNT/2];
}


//////////////////////////////////////////////////////////////////////////////////
// Destruction
//////////////////////////////////////////////////////////////////////////////////

Learning::~Learning()
{
	delete agent;
	delete net;
	delete bodyParts;
	outFile.close();
}

//////////////////////////////////////////////////////////////////////////////////
// public methods are following
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// this method is invoked every tick and checks where the program is and activates
// learning specific functions
//////////////////////////////////////////////////////////////////////////////////
void Learning::activateLearning(int k, float *nvalues)
{
	isTime = false;
	defineRelativePosition();
	
	if (!includeSensory)
		deleteValues();

	if (stepInitiated(k, agentCount))
	{
		if (!isRunning)
		{
			startTick = k;
			stopTick = k + STEPCYCLES;
		}
	}
	
	if (k == 2)
	{
		cout << endl << "Simulation started" << endl;
		cout << "walking..." << endl;
	}
	else if (fillingAngle == 0)
	{
		if (startTick > WAITCYCLES + (fillRuns * STEPCYCLES))
		{
			if (k >= startTick && k <= stopTick)
			{
				handleFillingStuff(k, startTick, stopTick);
			}
		}
	}
	else if (updated)
	{
		updated = false; //do not use this line if you want to use the following ones
		/*only necessary if you want to write the angles and motor neurons produced by the GA weights
		//writeAnglesMN(k, 0, nvalues);
		//learningStarted = 1;*/
	}
	else if (learning10finished != 0)
		writeAnglesMN(k, 10, nvalues);
	else if (learning100finished != 0)
		writeAnglesMN(k, 100, nvalues);
	else if (learning1000finished != 0)
		writeAnglesMN(k, 1000, nvalues);
	else if (learningStarted != 1)
	{
		if (startTick > WAITCYCLES + fillingAngle + ((learnRuns + 1) * STEPCYCLES) + offset)
		{
			if (k >= startTick && k <= stopTick)
			{
				handleLearningStuff(k, startTick, stopTick);
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////
// this method calls the appropriate functions if a new agent has been created
//////////////////////////////////////////////////////////////////////////////////
void Learning::configuringNewAgent(Agent *testAgent, float *svalues)
{
	if (SELECTION == 0)
		cout << "all sensory areas are allowed to learn" << endl;
	else if (SELECTION == 1)
		cout << "only spindle sensory areas are allowed to learn" << endl;
	else if (SELECTION == 2)
		cout << "only golgi sensory areas are allowed to learn" << endl;
	else if (SELECTION == 3)
		cout << "only cutaneous sensory areas are allowed to learn" << endl;
	else if (SELECTION == 4)
		cout << "oscillator areas are allowed to learn" << endl;

	allocateAgent(testAgent);
	
	writeToFile("old Weights.dat");

	if (updateWithGAWeights)
	{
		readFromFile(gaweights);
		changeToGAWeights();
		writeToFile("ga Weights.dat");
	}

	if (includeSensory)
	{
		net->NtfSensorRead(svalues);
		SENSOR_ACTIVATE = true;
	}
}


//////////////////////////////////////////////////////////////////////////////////
//this method returns true if it is time to delete and recreate mathWorld and testAgent in Grow
//////////////////////////////////////////////////////////////////////////////////
bool Learning::isTimeToRecreate(void)
{
	if (isTime == 1) //necessary because it gets sometimes 88 w/o obvious reason
	{
		fillingEnded = true;
		return true;
	}
	else
		return false;
}


//////////////////////////////////////////////////////////////////////////////////
// private methods are following
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
//this method provides the changing of the connection weights depending on the reward
//////////////////////////////////////////////////////////////////////////////////
void Learning::learnWeights(int k)
{
	int counter = 0;
	int counter2 = 0;
	k = (k - startTick) % STEPCYCLES;	//k defines the tick number relating to the step cycle

	// by the first invoking, just save actual hip angles (in getReward)
	// and the actual direction of foot movement; do not change values
	if (firstTime)
	{
		//variable initialization
		difference = 0.0;
		aa = 0.0;
		subtractionLeft = 0.0;
		subtractionRight = 0.0;
		oldHipLeft = 0.0;
		oldHipRight = 0.0;

		getReward(k);
		getValue(k);
		firstTime = false;
		return;
	}

	rewardValue = getReward(k);

	if (k % RESTRAINT == RESTRAINT - 1) //only every RESTRAINT tick
	{
		counter = 0;
		counter2 = 0;

		for (int x = 0; x < CONNECTIONCOUNT; x++)
		{
			// only for those connections wanted; SELECTION from constants.h defines the 
			// exact part of the network which is allowed to learn
			if (checkForAffiliation(x, SELECTION))
			{
				if (isConnectionActive(x)) //only if connection was active
				{
					counter++;
					updateWeights(rewardValue, x);
				}
				else	// forgetting	
				{
					counter2++;
					forget(x);
				}
			}
		}
		
		if (rewardValue > 0)
			cout << "this step was rewarded" << endl;
		else if (rewardValue == 0)
			cout << "this step was neutral" << endl;
		else
			cout << "this step was penaltied" << endl;
		cout << counter << " connections have learned" << endl;
		cout << counter2 << " connecticons have forgotten" << endl;
	}
	
	if (valueInclude) //calculate value function; is momentarily off
	{
		if (getValue(k) != 0)
		{
			for (int x = 0; x < CONNECTIONCOUNT; x++)
			{
				newStrength = net->getConnectionStrength(x) + getValue(k)*VALUE_RATE;
				net->setConnectionStrength(x, newStrength);
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
//this method performs the real weight changing of the connection; invoked by learnWeights()
//////////////////////////////////////////////////////////////////////////////////
void Learning::updateWeights(double rewardValue, int x)
{
	double ai = net->getTargetActivation(x);
	double aj = net->getSourceActivation(x);
	oldStrength = net->getConnectionStrength(x);

	if (rewardValue > 0) //if reward is provided
	{
		eta = LEARNING_RATE;
		
		// Hebbian learning algorithm
		if(oldStrength > 0)
			newStrength = oldStrength + eta * ai * aj;
		else
			newStrength = oldStrength - eta * ai * aj;
				
		net->setConnectionStrength(x, newStrength);
		
		if (debug)
		{
			cout << "connection from unit " << net->getSourceUnitID(x);
			cout << " to unit " << net->getTargetUnitID(x) << " has changed by ";
			cout << eta*100 << "%" << endl << " old weight: " << oldStrength;
			cout << ", new weight: " << net->getConnectionStrength(x) << endl;
		}
	}
	else if (rewardValue < 0) // if penalty is provided
	{
		// differentiate if angles were smaller or larger than ideal angles
		if (smaller)
			eta = (-1) * rewardValue*LEARNING_RATE;
		else
			eta = rewardValue*LEARNING_RATE;
			
		// Hebbian learning algorithm
		if(oldStrength > 0)
			newStrength = oldStrength + eta * ai * aj;
		else
			newStrength = oldStrength - eta * ai * aj;

		net->setConnectionStrength(x, newStrength);
		
		if (debug)
		{
			cout << "connection from unit " << net->getSourceUnitID(x) << " to unit " ;
			cout << net->getTargetUnitID(x) << " has changed by ";
			cout << eta*100 << "%" << endl << " old weight: " << oldStrength;
			cout << ", new weight: " << net->getConnectionStrength(x) << endl;
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////
//this method provides the forgetting by diminishuing the connection weights
//////////////////////////////////////////////////////////////////////////////////
void Learning::forget(int x)
{
	double ai = net->getTargetActivation(x);
	double aj = net->getSourceActivation(x);
	oldStrength = net->getConnectionStrength(x);
	eta = FORGETTING_RATE;

	if(oldStrength > 0)
		newStrength = oldStrength * (1 - eta);
	else
		newStrength = oldStrength * (1 - eta);

	net->setConnectionStrength(x, newStrength);
	
	if (debug)
	{
		cout << "connection from unit " << net->getSourceUnitID(x);
		cout << " to unit " << net->getTargetUnitID(x) << " has forgotten by ";
		cout << eta*100 << "%" << endl << " old weight: " << oldStrength;
		cout << ", new weight: " << net->getConnectionStrength(x) << endl;
	}
}


//////////////////////////////////////////////////////////////////////////////////
//this method returns a reward or penalty depending on the actual movement
//////////////////////////////////////////////////////////////////////////////////
float Learning::getReward(int k)
{
	float returnValue;
	newHipLeft = 0.0;
	newHipRight = 0.0;

	//version 1: reward is provided if actual joint angles are in a certain range
	//			 around a previously saved angle pattern
	if (REWARD_VERSION == 1)
	{
		for (int x = 0; x < JOINTS; x++)
		{
			aa = agent->getJointAngle(x);

			// if actual angle is smaller than ideal angle -> subtraction
			// else addition to difference
			if (angle[AVERAGE][k][x] >= 0)
				difference += aa - angle[AVERAGE][k][x];
			else
				difference += angle[AVERAGE][k][x] - aa;
		}
		
		if (difference > RESTRAINT*REWARD_AMOUNT)
		{
			smaller = false;
			returnValue = (float)-0.1;
		}
		else if (difference > RESTRAINT*REWARD_AMOUNT*(-1))
			returnValue = 1;
		else
		{
			smaller = true;
			returnValue = (float)-0.1;
		}
	}
	//version 2: reward is provided if the directions of movement of the 2 hip joints are opposite
	else if (REWARD_VERSION == 2)
	{
		newHipLeft = agent->getJointAngle(LEFT_HIP_ID);
		subtractionLeft += oldHipLeft - newHipLeft;
		oldHipLeft = newHipLeft;
		
		newHipRight = agent->getJointAngle(RIGHT_HIP_ID);
		subtractionRight += oldHipRight - newHipRight;
		oldHipRight = newHipRight;

		//maybe differentiation needed: only if subtraction has a certain value
		//which remains to be set
		if ((subtractionLeft > 0 && subtractionRight < 0) || (subtractionLeft < 0 && subtractionRight > 0))
			returnValue = 1;
		else
			returnValue = (float)-0.1;
	}
	else
		returnValue = 0;

//	if (debug)
		if (k % RESTRAINT == RESTRAINT - 1)
		{
			cout << "difference after "<< RESTRAINT << " ticks ";
			cout << "at step " << k+1 <<": " << difference << endl;
		}

	if (k % RESTRAINT == 0)
	{
		difference = 0.0;
		subtractionLeft = 0.0;
		subtractionRight = 0.0;
	}
	return returnValue;
}


//////////////////////////////////////////////////////////////////////////////////
//this method provides a value function; not to be implemented for the moment
//////////////////////////////////////////////////////////////////////////////////
int Learning::getValue(int k)
{
	return 0;
}


//////////////////////////////////////////////////////////////////////////////////
// this method records the angles during some step cycles and averages them before
// learning is started
//////////////////////////////////////////////////////////////////////////////////
void Learning::fillAngle(int k, short s)
{	
	for (int x = 0; x < JOINTS; x++)
		angle[s][k-1][x] = agent->getJointAngle(x);

	if (k == 0)
	{
		cout << endl << "filling angle started at step " << k + startTick << endl;

		if (agentDetPos)
		{
			cout << "foot: " << agent->GetJointPosition(RIGHT_ANKLE_ID);
			cout << "; knee: " << agent->GetJointPosition(RIGHT_KNEE_ID) << endl;
		}
		else
		{
			MdtBodyGetPosition(bodyParts[RIGHT_ANKLE_ID], jointPosition);
			cout << "foot: " << jointPosition[2];
			MdtBodyGetPosition(bodyParts[RIGHT_KNEE_ID], jointPosition);
			cout << "; knee: " << jointPosition[2] << endl;
		}

		cout << "filling..." << endl;
	}
	
	if (k == STEPCYCLES)
	{
		cout << "angle is filled at step " << k + startTick;
		cout << "; run " << s + 1 << " of " << AVERAGE << endl << endl;

		if (s == AVERAGE - 1)
		{
			cout << "calculating average joint angle..." << endl;
			for (int y = 0; y < JOINTS; y++)
			{
				for (int z = 0; z < STEPCYCLES; z++)
				{
					for (int a = 0; a < AVERAGE; a++)
						temp += angle[a][z][y];
					angle[AVERAGE][z][y] = temp/AVERAGE;
					temp = 0;
				}
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////
//this method handles the code which is necessary for recording the angle pattern
//////////////////////////////////////////////////////////////////////////////////
void Learning::handleFillingStuff(int k, int startTick, int stopTick)
{
	isRunning = true;
	fillAngle(k - startTick, fillRuns);

	if (s == 0)
	{
		if (agentDetPos)
			jointPosition[2] = agent->GetJointPosition(RIGHT_KNEE_ID);
		else
			MdtBodyGetPosition(bodyParts[RIGHT_KNEE_ID], jointPosition);
		kneePos = jointPosition[2];
		s++;
	}

	if (k == stopTick)
	{
		fillRuns++;
		isRunning = false;
	}

	if (k == stopTick && fillRuns == AVERAGE)
	{
		fillingAngle = stopTick;
		fillingMode = false;

		if (letAgentGrow)
		{
			agentCount++;
			adjustBodySpecificConstants();
			isTime = true;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
//this method handles the code which is necessary for learning (but not the learning itself)
//////////////////////////////////////////////////////////////////////////////////
void Learning::handleLearningStuff(int k, int startTick, int stopTick)
{
	isRunning = true;

	// if first tick of a step cycle
	if (k == startTick)
	{
		cout << endl << "learning started at step " << k << endl;

		if (agentDetPos)
		{
			cout << "foot: " << agent->GetJointPosition(RIGHT_ANKLE_ID);
			cout << "; knee: " << agent->GetJointPosition(RIGHT_KNEE_ID) << endl;
		}
		else
		{
			MdtBodyGetPosition(bodyParts[RIGHT_ANKLE_ID], jointPosition);
			cout << "foot: " << jointPosition[2];
			MdtBodyGetPosition(bodyParts[RIGHT_KNEE_ID], jointPosition);
			cout << "; knee: " << jointPosition[2] << endl;
		}
	}

	// every tick of a step cycle
	learnWeights(k);

	// if last tick of a step cycle
	if (k == stopTick)
	{
		offset = startTick - (WAITCYCLES + fillingAngle + (learnRuns * STEPCYCLES));
		learnRuns++;
		isRunning = false;
		cout << "learning finished at step " << k;
		cout << "; run " << learnRuns << " of " << LEARNING_STEPS << endl << endl;

		if (learnRuns == 10)
		{
			cout << "writing data..." << endl;
			learningStarted = 1;
			learning10finished = true;
		}
		else if (learnRuns == 100)
		{
			cout << "writing data..." << endl;
			learningStarted = 1;
			learning100finished = true;
		}
		else if (learnRuns == 1000)
		{
			cout << "writing data..." << endl;
			learningStarted = 1;
			learning1000finished = true;
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////
//this method allocates the testAgent to the agent used inside the learning class
//////////////////////////////////////////////////////////////////////////////////
void Learning::allocateAgent(Agent *testAgent)
{
	agent = testAgent;
	bodyParts = agent->bodyParts;
}


//////////////////////////////////////////////////////////////////////////////////
//this method returns true if a step cycle has been initiated
//////////////////////////////////////////////////////////////////////////////////
bool Learning::stepInitiated(int k, int count)
{
	if (k < count * WAITCYCLES)	// give the agent time to get to its walking pattern
		return false;
	
	if (securityCheck == STEPCYCLES + 1)
		securityCheck = 0;
	if (securityCheck != 0)
		securityCheck++;
	
	if (hasZeroPassed())	// if a complete step cycle has been conducted
	{
		if (securityCheck == 0)
		{
			securityCheck++;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}


//////////////////////////////////////////////////////////////////////////////////
// this method returns if the ankle has passed the zero position
//////////////////////////////////////////////////////////////////////////////////
bool Learning::hasZeroPassed(void)
{
	if (agentDetPos)
	{
		if (akp = agent->GetJointPosition(RIGHT_KNEE_ID) > 0)	// if knee is already swept forward
		{
			if (fillingMode)
				return switched;
			else
			{
				akp = jointPosition[2];

				if ((kneePos*(100-POS_DEVIATION)/100 < akp) && (akp < kneePos*(100+POS_DEVIATION)/100))
					return switched;
				else
					return false;
			}
		}
		else
			return false;
	}
	else
		return switched;	// knee control function doesn't work with MdtGetJointPosition
}

//////////////////////////////////////////////////////////////////////////////////
// this method checks the belonging of a certain connection to the defined learning
// area (defined by SELECTION)
//////////////////////////////////////////////////////////////////////////////////
bool Learning::checkForAffiliation(int x, short selection)
{
	int ID = net->getSourceAreaID(x);

	if (selection == 0)	// all sensory areas
	{
		//areas 16-46 and 48-60 are part of the sensory system
		if (ID >= 16 && ID <= 60 && ID != 47)
			return true;
		else
			return false;
	}
	else if (selection == 4) // oscillator areas
	{
		switch (ID)
		{
			case 0:
			case 1:
			case 6:
			case 7:
			case 11:
			case 12:
				return true;
		}
		return false;
	}
	else
	{
		ID = net->getSourceUnitID(x);

		if (selection == 1) // only spindle areas
		{
			switch (ID)
			{
				case 390:
				case 392:
				case 399:
				case 403:
				case 405:
				case 406:
				case 414:
				case 417:
				case 475:
				case 476:
				case 485:
				case 483:
				case 504:
				case 505:
				case 497:
				case 499:
				case 554:
				case 555:
				case 548:
				case 546:
				case 564:
				case 567:
				case 561:
				case 557:
					return true;
			}

			if (ID >= 3197 && ID <= 3208)
				return true;
			else if (ID >= 5200 && ID <= 5211)
				return true;
			else if (ID >= 1533 && ID <= 1544)
				return true;
			else
				return false;
		}
		else if (selection == 2) // only golgi areas
		{
			switch (ID)
			{
				case 391:
				case 393:
				case 407:
				case 408:
				case 473:
				case 474:
				case 502:
				case 503:
				case 552:
				case 553:
				case 565:
				case 566:
					return true;
			}

			if (ID >= 1521 && ID <= 1232)
				return true;
			else
				return false;
		}
		else if (selection == 3) // only cutaneous areas
		{
			switch (ID)
			{
				case 394:
				case 409:
				case 419:
				case 420:
				case 1545:
				case 1546:
				case 3196:
					return true;
			}

			if (ID >= 630 && ID <= 639)
				return true;
			else
				return false;
		}
		else
			return false;
	}
}


//////////////////////////////////////////////////////////////////////////////////
//this method returns true if the connection between 2 units is active
//////////////////////////////////////////////////////////////////////////////////
bool Learning::isConnectionActive(int x)
{
	for (int i = 0; i < UNITCOUNT; i++)
	{
		if (net->Unit[i].UI == net->getSourceUnitID(x))
			sourceActivity = net->Unit[i].Value_old;
		else if (net->Unit[i].UI == net->getTargetUnitID(x))
			targetActivity = net->Unit[i].Value_old;
	}
	if (sourceActivity != 0)
		cout << "";

	if (sourceActivity > 0.0 && targetActivity > 0.0)
		return true;
	else
		return false;
}


//////////////////////////////////////////////////////////////////////////////////
// this method defines the relative position of the ankle to zero
// and sets the switched variable
//////////////////////////////////////////////////////////////////////////////////
void Learning::defineRelativePosition(void)
{
	if (agentDetPos)
		jointPosition[2] = agent->GetJointPosition(RIGHT_ANKLE_ID);
	else
		MdtBodyGetPosition(bodyParts[RIGHT_ANKLE_ID], jointPosition);

	if (jointPosition[2] > 0)
	{
		if (belowZero)
		{
			if (stepcyclePrintout)
			{
				cout << "one stepcycle = " << stepcounter << " ticks" << endl;
				stepcounter = 0;
			}
			switched = true;
		}
		else
			switched = false;
		overZero = true;
		belowZero = false;
	}
	else
	{
		belowZero = true;
		overZero = false;
		switched = false;
	}

	if (debug)
		stepcounter++;
}


//////////////////////////////////////////////////////////////////////////////////
// this method sets the sensory part weights to new values gained by a GA from Andi
//////////////////////////////////////////////////////////////////////////////////
void Learning::changeToGAWeights(void)
{
	int g = 0;
	for (int number = 0; number < CONNECTIONCOUNT; number++)
	{		
		if(checkForAffiliation(number, 0))
		{
			net->setConnectionStrength(number, geneWeights[g]);
			g++;
		}
	}
	cout << g << " connections updated with GA weights" << endl;
	updated = true;
}

//////////////////////////////////////////////////////////////////////////////////
// this method sets the Value_old of some units to 0 if includeSensory is false
// reason: have positive activation in this case although they're part of sensory system
//////////////////////////////////////////////////////////////////////////////////
void Learning::deleteValues(void)
{
	for (int i = 0; i < UNITCOUNT; i++)
	{
		switch(net->Unit[i].UI)
		{
			case 439:
			case 462:
			case 464:
			case 469:
			case 479:
			case 490:
			case 491:
			case 506:
			case 540:
			case 541:
			case 569:
			case 571:
				net->Unit[i].Value_old = 0.0;		
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////
// this method adjusts the joint IDs according to which kind of joint position 
// measuring is chosen. This can't be done in the constructor, because the boolean
// variable changes its given value
//////////////////////////////////////////////////////////////////////////////////
void Learning::setJointIDs(void)
{
	if (agentDetPos)
		RIGHT_ANKLE_ID = 4;
	else
		RIGHT_ANKLE_ID = 1;
}


//////////////////////////////////////////////////////////////////////////////////
// this method adjusts the body specific constants from constants.h for the new agent
//////////////////////////////////////////////////////////////////////////////////
void Learning::adjustBodySpecificConstants(void)
{
	HEIGHT_OFFSET		=	0.1;

	// change the DEFAULT_RADIUS by AGENT_CHANGE percent
	DEFAULT_RADIUS		=	DEFAULT_RADIUS * ((100 + AGENT_CHANGE)/100);

	// adjust the variables depending from DEFAULT_RADIUS
	DEF_CONNECTOR_MASS	=	DEFAULT_RADIUS * 20;
	DEF_LL_LENGTH		=	DEFAULT_RADIUS * 8;
	DEF_UL_LENGTH		=	DEF_LL_LENGTH;
	DEF_LL_MASS			=	DEF_LL_LENGTH * 10;
	DEF_UL_MASS			=	DEF_UL_LENGTH * 20;
	DEF_LL_WIDTH		=	DEFAULT_RADIUS / 2;
	DEF_UL_WIDTH		=	DEFAULT_RADIUS / 4 * 3;
	DEF_FOOT_LENGTH		=	DEFAULT_RADIUS * 4;
	DEF_FOOT_MASS		=	DEF_FOOT_LENGTH * 20;
	DEF_FOOT_WIDTH		=	DEFAULT_RADIUS * 2;
	DEF_FOOT_THICKNESS	=	DEFAULT_RADIUS / 2;
	BODY_WIDTH			=	DEF_LL_LENGTH;
	DEF_WAIST_MASS		=	BODY_WIDTH * 70 * 50;
	DEF_WAIST_WIDTH		=	DEFAULT_RADIUS;

	// adjust virtual world specific DEFAULT_RADIUS dependent constants
	TARGET_SIZE			=	DEFAULT_RADIUS;
}


//////////////////////////////////////////////////////////////////////////////////
// this method prepares the joint angle and MN activity data for file output
//////////////////////////////////////////////////////////////////////////////////
void Learning::writeAnglesMN(int k, int m, float *nvalues)
{
	if (mnOutput)
	{
		if (outputCounter < 3 * STEPCYCLES)
		{
			for (int i = 0; i < JOINT_ANGLES_COUNT/2; i++)
			{	
				jointAngles[i] = agent->getJointAngle(i);

				if ((i == 0)||(i == 3))
					jointAngles[i]= jointAngles[i] - 0.572801;
				else if (i == 1)
					jointAngles[i] = (jointAngles[i] + 0.0579246) * -1;
				else if (i == 2)
					jointAngles[i] = (jointAngles[i] - 0.057459);
				else if ((i == 4)||(i == 5))
					jointAngles[i] = (jointAngles[i] - 0.0006) * -1;
			}

			for (i = 0; i < MOTORCOUNT + JOINT_ANGLES_COUNT/2; i++)
			{
				if (i < MOTORCOUNT)
					outputValues[i] = nvalues[i];
				else
					outputValues[i] = jointAngles[i - MOTORCOUNT];
			}

			if (k > 0)
				writetoFile(k, m, outputValues);

			outputCounter++;
		}
		else if (outputCounter == 3 * STEPCYCLES)
		{
			if (learning10finished == 1)
			{
				writeToFile("10ls_new Weights.dat");//write connections into output file
				learning10finished = 0;
				learningStarted = 0;
			}
			else if (learning100finished == 1)
			{
				writeToFile("100ls_new Weights.dat");//write connections into output file
				learning100finished = 0;
				learningStarted = 0;
			}
			else if (learning1000finished == 1)
			{
				writeToFile("1000ls_new Weights.dat");//write connections into output file
				learning1000finished = 0;
				cout << "Simulation progressing with new weights" << endl;
			}
			else if (updated)
			{
				learningStarted = 0;
				updated = false;
			}

			outputCounter = 0;
		}
	}
	else
		return;
}


//////////////////////////////////////////////////////////////////////////////////
// I/O methods are following
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// reads the weights of a GA from an extern file and copies them into an array to
// overwrite sensory part weights of the net
//////////////////////////////////////////////////////////////////////////////////
void Learning::readFromFile(char *name)
{
	int length = 0;
	char inFileName[50];

	sprintf(inFileName, name);
	
	inFile.open(inFileName);
	inFile >> length;
	geneWeights = new float[length];

	for (int g = 0; g < length; g++)
		inFile >> geneWeights[g];

	inFile.close();
}


//////////////////////////////////////////////////////////////////////////////////
// this method writes the connection weights into a file
//////////////////////////////////////////////////////////////////////////////////
void Learning::writeToFile(char name[])
{
	char outFileName[50];

	sprintf(outFileName, "data/%s", name);

	outFile.open(outFileName);

	for (int x = 0; x < CONNECTIONCOUNT; x++)
		outFile << net->getConnectionStrength(x) << "\n";

	outFile.close();
}


//////////////////////////////////////////////////////////////////////////////////
// this method writes the joint angles and the MN activities into a file
//////////////////////////////////////////////////////////////////////////////////
void Learning::writetoFile(int ID, int m, double channelValues[])
{
	char outFileName[50];

	sprintf(outFileName,"data/%dls_%d.dat", m, ID);

	outFile.open(outFileName);

	for (int x = 0; x < MOTORCOUNT + JOINT_ANGLES_COUNT/2; x++)
		outFile << channelValues[x] << "\n";

	outFile.close();
}


#endif