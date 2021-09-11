//////////////////////////////////////////////////////////////////////
// Learning.h: Interface for class Learning
// Author    : Daniel Baumgartner
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_LEARNING_H__F3570235_5D71_4637_852B_9DB67A9D1301__INCLUDED_)
#define AFX_LEARNING_H__F3570235_5D71_4637_852B_9DB67A9D1301__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "NeuralNet.h"
#include "Agent.h"


class Learning  
{
// methods
public:
	Learning(Agent *testAgent, NeuralNet *neuralNet);
	virtual ~Learning();
	void	activateLearning(int k, float *nvalues);
	void	configuringNewAgent(Agent *testAgent, float *svalues);
	bool	isTimeToRecreate(void);

private:
	float	getReward(int k);
	int		getValue(int k);

	void	learnWeights(int k);
	void	writeToFile(char name[]);
	void	writetoFile(int ID, int m, double channelValues[]);
	void	fillAngle(int k, short s);
	void	allocateAgent(Agent *testAgent);
	void	defineRelativePosition(void);
	void	adjustBodySpecificConstants(void);
	void	readFromFile(char *name);
	void	changeToGAWeights(void);
	void	setJointIDs(void);
	void	updateWeights(double rewardValue, int x);
	void	handleFillingStuff(int k, int startTick, int stopTick);
	void	handleLearningStuff(int k, int startTick, int stopTick);
	void	deleteValues(void);
	void	forget(int x);
	void	writeAnglesMN(int k, int m, float *nvalues);

	bool	stepInitiated(int k, int count);
	bool	hasZeroPassed(void);
	bool	checkForAffiliation(int x, short selection);
	bool	isConnectionActive(int x);


// variables
private:
	int		startTick;
	int		stopTick;
	int		securityCheck;			// security variable which guarantees length of a step cycle
	int		agentCount;				// counts the number of agents
	int		oldCount;
	int		fillingAngle;
	int		stepcounter;			// counts how many ticks make 1 stepcycle
	int		offset;					// offset of the starting tick of learning cycle to calculated one
	int		outputCounter;
	
	bool	updated;
	bool	firstTime;
	bool	isTime;
	bool	overZero;
	bool	belowZero;
	bool	switched;
	bool	fillingMode;
	bool	isRunning;				// true if it is filling angle pattern or learning
	bool	smaller;				// true if actual joint angles are smaller than ideal ones
	bool	fillingEnded;			// true if the initial recording of the joint angles has been finished
	bool	learning10finished;		// true if first 10 learn steps are finished
	bool	learning100finished;	// true if first 100 learn steps are finished
	bool	learning1000finished;	// true if first 1000 learn steps are finished
	
	double	rewardValue;
	double	newStrength;
	double	oldStrength;
	double	newDistance;			// program doesn't work w/o this variable, although it's nowhere been used
	double	rearmostPosRight;		// program doesn't work w/o this variable, although it's nowhere been used
	double	akp;					// actual knee position
	double	angle[6][264][6];		// contains the known angle pattern of a step cycle
									// IMPORTANT: depends on the constants STEPCYCLES, JOINTS and AVERAGE in constants.h
	double	aa;						// actual angle
	double	difference;
	double	eta;					// learning rate: combination of LEARNING_RATE with reward
	double	subtractionLeft;
	double	subtractionRight;
	double	newHipRight, newHipLeft;
	double	oldHipRight, oldHipLeft;
	double	newFoot, oldFoot;
	double	kneePos;				// position of right knee in the moment of ankle crossing origo
	double	temp;					// a help variable
	double	sourceActivity, targetActivity;

	float	*geneWeights;		// the weights from ga0.dat are copied into this array
	double	*jointAngles;
	double	*outputValues;
	
	short	halfStepCycle;
	short	newMovementDirection;	// 1 = forward, -1 = backward
	short	oldMovementDirection;
	short	learningStarted;
	short	s;						// a counter for setting knee position
	short	fillRuns;				// a counter for the filling of the angle array
	short	learnRuns;				// counts in which learning cycle the agent is
	
	Agent		*agent;
	NeuralNet	*net;
	ofstream	outFile;
	ifstream	inFile;
	MeVector3	jointPosition;
	MdtBodyID	*bodyParts;

};

#endif