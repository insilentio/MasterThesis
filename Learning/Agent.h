/* ---------------------------------------------------
   FILE:     agent.h
	AUTHOR:   Josh Bongard, Chandana Paul
	DATE:     October 3, 2000
	FUNCTION: This class contains all information for
				 a single agent, complete with morphology
				 and neural control, and the MathEngine
				 specific code.
 -------------------------------------------------- */

#include "stdafx.h"
#include "fstream.h"
#include "iostream.h"

#ifndef _AGENT_H
#define _AGENT_H

#include "McdFrame.h"
#include "Mdt.h"
#include "MeViewer.h"

class Agent {

public:
	MdtBodyID				*bodyParts;	//because it is used inside learning

private:
	ofstream				bodyPlanFile;
	ofstream				neuralNetFile;
	MdtBodyID				target;
	RGraphicsDescription	*bodyPartsG[13];
	RGraphicsDescription    *targetG;
	McdModelID				*bodyPartsCM;
	McdModelID              targetCM;
	McdGeometryID			*bodyParts_prim;
	McdGeometryID           target_prim;
	McdModelID				*bodyWaistCM;
	McdGeometryID			*bodyWaist_prim;
	MdtHingeID				joints[14];
	MdtPrismaticID			pjoint;
	MdtLimitID				*limits;
	ofstream				outFile;
	MeVector3				pos_vector_foot_left_new; // they do have to be setted to 0 first
	MeVector3				pos_vector_foot_right_new;
	float					pos_vector_foot_left_old;
	float					pos_vector_foot_right_old;
	float                   *move_value;
	float					*forces;
	float					*velocity;
	float				    lfc, rfc;
	float					temp[6];
	float					delta[6];
	
public:
	Agent(void);
	~Agent(void);
	int		GetFootContact(int i);
	float	GetJointPosition(int i);
	float	GetGolgiValues(int i);
	float	GetMuscleSpindleValues(int i);
	float	GetMuscleDynamicSpindleValues(int i);
	float	GetCutaneousValues(int i);
	float	getJointAngle(int jointID);
	void	Move(int g);
	void	TurnOffMotor(int i);
	void	TurnOnMotor(int i);
	void	SetJointMoveValue(int i, float Oj);
	void	SetJointMoveValue(int i, float Oj, float force);
	void	CreateTarget();
	void	DestroyTarget();
	int		*mode;	// 0 is position control
					// 1 is force control
					// 2 is turned off
					// 3 is cocontraction
	
					// 0: left knee, 1: left pitch, 2: left roll
					// 3: right roll 4: right pitch 5: right knee
  
private:
	void	ConstrainJoints(void);
	void	CreateBodyParts(void);
	void	CreateCylinders(void);
	void	CreateFeetKneesHips(void);
	void	CreateJoints(void);
	void	DestroyBodyParts(void);
	void	DestroyCylinders(void);
	void	DestroyFeetKneesHips(void);
	void	DestroyJoints(void);
	void	SetJointLimits(int jointIndex, float limMin, float limMax);
	void	WeldJoint(int jointIndex);

};

#endif