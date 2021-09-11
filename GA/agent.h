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

class AGENT {

private:
	ofstream bodyPlanFile;
	ofstream neuralNetFile;
	MdtBodyID				 *bodyParts;
	MdtBodyID				 target;
	RGraphicsDescription	 *bodyPartsG[13];
	RGraphicsDescription     *targetG;
	McdModelID				 *bodyPartsCM;
	McdModelID               targetCM;
	McdGeometryID			 *bodyParts_prim;
	McdGeometryID            target_prim;
	McdModelID				 *bodyWaistCM;
	McdGeometryID			 *bodyWaist_prim;



	MdtHingeID				 joints[14];
	MdtPrismaticID			 pjoint;
	MdtLimitID				 *limits;
	ofstream				 outFile;
	MeVector3				 pos_vector_foot_left_new; // they do have to be setted to 0 first
	float					 pos_vector_foot_left_old;
	MeVector3				 pos_vector_foot_right_new;
	float					 pos_vector_foot_right_old;
	float					 max_fw_pos_left;
	float					 max_fw_pos_right;
	float					 max_bw_pos_left;
	float					 max_bw_pos_right;
	double					 max_fw_angle_left;
	double					 max_bw_angle_left;
	double					 max_fw_angle_right;
	double					 max_bw_angle_right;
	double					 angle_difference_left;
	double					 angle_difference_right;
	float                    *move_value;
	float					 *forces;
	float					 *velocity;
	float				     lfc, rfc;
	int						 leftStep;
	int						 rightStep;
	int                      runs;
	int						directionBuffer;
	float                   fitness;
	//2 arrays for method GetMuscleSpindleValues to save old joint position value
	//in order to calculate delta after the next tick
	float					temp[6];
	float					delta[6];
	float					left_step_distance;
	float					right_step_distance;
	float					left_foot_distance; //to calculate the distance the feet travel on a virtual tredmill
	float					right_foot_distance;
	float					muscleactivity;
	bool					left_FORWARD_old;
	bool					left_FORWARD_new;
	bool					right_FORWARD_old;
	bool					right_FORWARD_new;
	bool					left_M1;
	bool					left_M2;
	bool					right_M1;
	bool					right_M2;
	bool					SWINGING_LEG;
	bool					isFirstLeft;
	bool					isFirstRight;
	bool					isSecondLeft;
	bool					isSecondRight;
	bool					outOfPhase;
	bool					directionChange_left;
	bool					directionChange_right;
	bool					HipAngleDiff;
	bool					directionChange;



	
public:
	AGENT(void);
	~AGENT(void);
	void Move(int g);
	float GetJointPosition(int i);
	void SetJointMoveValue(int i, float Oj);
	void SetJointMoveValue(int i, float Oj, float force);
	int GetFootContact(int i);
	void AGENT::TurnOffMotor(int i);
	void AGENT::TurnOnMotor(int i);
    float AGENT::GetFitness();
	float AGENT::GetGolgiValues(int i);
	float AGENT::GetMuscleSpindleValues(int i);
	float AGENT::GetMuscleDynamicSpindleValues(int i);
	float AGENT::GetCutaneousValues(int i);
	void AGENT::CreateTarget();
	void AGENT::DestroyTarget();
	void AGENT::MuscleactivityCalculate(float i);
	void AGENT::MuscleactivityReset();
	float AGENT::getJointAngle(int jointID);
	int *mode;	// 0 is position control
				// 1 is force control
				// 2 is turned off
				// 3 is cocontraction


   // 0: left knee, 1: left pitch, 2: left roll
   // 3: right roll 4: right pitch 5: right knee

   void AGENT::WritetoFile(int i, float channelValues[]);
   
private:
	void ConstrainJoints(void);
	void CreateBodyParts(void);
	void CreateCylinders(void);
	void CreateFeetKneesHips(void);
	void CreateJoints(void);
	void DestroyBodyParts(void);
	void DestroyCylinders(void);
	void DestroyFeetKneesHips(void);
	void DestroyJoints(void);
	void SetJointLimits(int jointIndex, float limMin, float limMax);
	void WeldJoint(int jointIndex);
	void PrepareWalkingDistanceCalculation();
	void SettingVariables();
	void SettingLeftFootVariables();
	void SettingRightFootVariables();
	void FeetForwardsOrBackwards();
	void FeetChangingDirection();
	void GetLeftFootMaximum();
	void GetRightFootMaximum();
	void LeftFootStep();
	void RightFootStep();
	void StepDistanceLeft();
	void StepDistanceRight();
	void StepDistanceReset();
	void OutOfPhase();
	void BufferIncrease();
	void DirectionControll();
	void AngleDifferenceLeft();
	void AngleDifferenceRight();
	bool AngleDiffCompare();

};

#endif