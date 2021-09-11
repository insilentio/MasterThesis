/* ---------------------------------------------------
	FILE:   agent.cpp
	AUTHOR:	Josh Bongard (modified by Chandana Paul,
			Andreas Durrer, Tobias Kueng, Daniel Baumgartner)
	DATE:   October 3, 2000
	FUNCTION: This class contains all information for
				 a single agent, complete with morphology
				 and neural control. MathEngine specific
				 code is also included.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _AGENT_CPP
#define _AGENT_CPP

#include "agent.h"
#include "simParams.h"
#include "mathWorld.h"

#include "McdDtBridge.h"
#include "Mdt.h"
#include "MeViewer.h"
#include "McdFrame.h"
#include "McdPrimitives.h"
#include "MeMath.h"

extern SimParams *simParams;

extern int			NUM_BODY_PARTS;
extern int			MAXNUMPAIRS;
extern MdtWorldID	world;
extern McdSpaceID   space;
extern McdDtBridgeID   cdHandler;
extern MdtBodyID    target;
extern float		red[3];
extern float		BODY_WIDTH;
extern float		DEFAULT_RADIUS;
extern float		DEF_CONNECTOR_MASS;
extern int			NUM_CONNECTORS;
extern RRender		*rc;
extern float		purple[3];
extern float		green[3];
extern float		DEF_LL_LENGTH;
extern float		DEF_LL_MASS;
extern float		DEF_LL_WIDTH;

extern float		DEF_FOOT_LENGTH;
extern float		DEF_FOOT_MASS;
extern float		DEF_FOOT_WIDTH;

extern float		DEF_UL_LENGTH;
extern float		DEF_UL_MASS;
extern float		DEF_UL_WIDTH;
extern float		DEF_FOOT_THICKNESS;

extern float		DEF_WAIST_MASS;
extern float		DEF_WAIST_WIDTH;
extern float		STILT_HEIGHT;

extern int			NUM_JOINTS;
extern float		HIP_PITCH_MINIMUM;
extern float		HIP_PITCH_MAXIMUM;
extern float		HIP_ROLL_MINIMUM;
extern float		HIP_ROLL_MAXIMUM;
extern float		KNEE_MINIMUM;
extern float		KNEE_MAXIMUM;
extern float		ANKLE_MINIMUM;
extern float		ANKLE_MAXIMUM;

extern float		DAMPING;
extern float		STIFFNESS;
extern int			FORCE_CONTROL;
extern int			lfootcontact, rfootcontact, waistcontact; 
extern int			leftFootMaterial, rightFootMaterial, floorMaterial, waistMaterial;

extern float		HEIGHT_OFFSET;
extern float		TARGET_SIZE;
extern float		TARGET_DIST;

extern int			DATACOUNT;

extern float	MarkPoint1;
extern float	MarkPoint2;
extern short      EXP;
extern bool		firstAgent;
int nextstep, a;

/* ---------------------------------------------------
							CALLBACK FUCNTIONS
 -------------------------------------------------- */

unsigned int LeftFootContactCB(MdtContactID mdtcontact, McdContact *mcdcontact, McdIntersectResult *result)
{
	lfootcontact = 1; 
	return 1; 
}

unsigned int RightFootContactCB(MdtContactID mdtcontact, McdContact *mcdcontact, McdIntersectResult *result)
{
	rfootcontact = 1; 
	return 1; 	
}

/*---------------------------------------------------------------
							Kollision zwischen Roboter und Würfel
---------------------------------------------------------------*/
unsigned int WaistContactCB(MdtContactID mdtcontact, McdContact *mcdcontact, McdIntersectResult *result)
{
	waistcontact = 1; 
	return 1; 	
}

/* ---------------------------------------------------
							PUBLIC METHODS
 -------------------------------------------------- */


Agent::Agent(void)
{
	if(firstAgent)
	{
		printf("Creating an agent.\n");
		firstAgent = false;
	}
	else
		printf("recreating an agent.\n");

	CreateBodyParts();
	move_value = new float[6];
	for(int i = 0; i < 6; ++i)
		move_value[i] = 0;
	velocity = new float[6];
	for(i = 0; i < 6; ++i)
		velocity[i] = 0;
	mode = new int[6];
		for(i = 0; i < 6; ++i)
		mode[i] = 0;
	forces = new float[6];
	for(i = 0; i < 6; ++i)
		forces[i] = 0.0; 
	for (i=0; i<3; i++)
	{
		pos_vector_foot_left_new [i] = 0.0; 
		pos_vector_foot_right_new [i] = 0.0;
	}

	pos_vector_foot_left_old = 0.0;
	pos_vector_foot_right_old = 0.0;

	//setting temp[i] to zero to have a start value before the first tick
	temp[i] = 0.0;
	a =0;
}

Agent::~Agent(void)
{
	delete[] move_value;
	DestroyBodyParts(); 
}

void Agent::Move(int g)
{
	MdtBodyGetPosition(bodyParts[0], pos_vector_foot_left_new);// get position of the both feet
	MdtBodyGetPosition(bodyParts[1], pos_vector_foot_right_new);

	int jnt[8];

	jnt[0] = 8; // right knee
	jnt[1] = 6; // right hip roll
	jnt[2] = 3; // left hip roll
	jnt[3] = 1; // left knee
	jnt[4] = 9; // right ankle
	jnt[5] = 0; // left ankle
	jnt[6] = 4; // left pitch
	jnt[7] = 5; // right pitch

	int currHingeJoint;

	// reset footcontact sensors
	if (lfootcontact == 1)
	{
		lfc = 1.0;
		lfootcontact = 0;
	}
	else if (lfootcontact == 0)
		lfc = 0.0;

	if (rfootcontact == 1) {
		rfc = 1.0;
		rfootcontact = 0;
	}
	else if (rfootcontact == 0)
		rfc = 0.0;

	for ( currHingeJoint=0;currHingeJoint<6;currHingeJoint++)
	{
		int j = jnt[currHingeJoint];	
		limits[currHingeJoint] = MdtHingeGetLimit(joints[j]);
		float error;
			
		if (MdtLimitIsMotorized(limits[currHingeJoint]))
		{
			if (FORCE_CONTROL == 0) {
			float error = (move_value[currHingeJoint] - MdtLimitGetPosition(limits[currHingeJoint]));
			MdtLimitSetLimitedForceMotor(limits[currHingeJoint], 3*error, 600.0); // Multiplikation mit Error hatt schnellere Bewegungsausführung zur Folge
			}
			else { switch(mode[currHingeJoint]) {
				case 0: //position control at 0
				MdtLimitSetLimitedForceMotor(limits[currHingeJoint], 0.0,
					600.0*move_value[currHingeJoint]);
				break;
				case 1: // force control
				if (move_value[currHingeJoint] > 0.0) {
				MdtLimitSetLimitedForceMotor(limits[currHingeJoint], 
					5.0 - MdtLimitGetVelocity(limits[currHingeJoint]), 
				   600.0*move_value[currHingeJoint]);
					}
				   else {
				MdtLimitSetLimitedForceMotor(limits[currHingeJoint], 
					-5.0 - MdtLimitGetVelocity(limits[currHingeJoint]), 
					-600.0*move_value[currHingeJoint]);	
					}
				   break;	
				case 2: 
					if (MdtLimitGetVelocity(limits[currHingeJoint]) > 2.0)
					MdtLimitSetLimitedForceMotor(limits[currHingeJoint], 0, 600.0);
					break;
				case 3: 
					error = (move_value[currHingeJoint] - MdtLimitGetPosition(limits[currHingeJoint]));
					MdtLimitSetLimitedForceMotor(limits[currHingeJoint], 3*error, forces[currHingeJoint]*600.0); 
					break;	
				case 4: 
					error = (move_value[currHingeJoint] - MdtLimitGetPosition(limits[currHingeJoint]));
					MdtLimitSetLimitedForceMotor(limits[currHingeJoint], 3*error, forces[currHingeJoint]*600.0); 
					break;
				}
			}
		}
	}
}

void Agent::TurnOffMotor(int currHingeJoint)
{
	int jnt[8];

	jnt[0] = 8; // right knee
	jnt[1] = 6; // right hip roll
	jnt[2] = 3; // left hip roll
	jnt[3] = 1; // left knee
	jnt[4] = 9; // right ankle
	jnt[5] = 0; // left ankle
	jnt[6] = 4; // left pitch
	jnt[7] = 5; // right pitch

	int j = jnt[currHingeJoint];	
	MdtLimitID	limit = MdtHingeGetLimit(joints[j]);
	MdtLimitActivateMotor(limit, 0);  // deactivate motor
	mode[currHingeJoint] = 2;
}

void Agent::TurnOnMotor(int currHingeJoint)
{
	int jnt[8];

	jnt[0] = 8; // right knee
	jnt[1] = 6; // right hip roll
	jnt[2] = 3; // left hip roll
	jnt[3] = 1; // left knee
	jnt[4] = 9; // right ankle
	jnt[5] = 0; // left ankle
	jnt[6] = 4; // left pitch
	jnt[7] = 5; // right pitch

	int j = jnt[currHingeJoint];	

	MdtLimitID	limit = MdtHingeGetLimit(joints[j]);
	MdtLimitActivateMotor(limit, 1);  // activate motor
}

float Agent::GetJointPosition(int i)
{
	//0: left knee 1: left roll 2: right roll 3: right knee 4: left ankle 5: right ankle

	if ((i < 0) || (i > 5))
	{
		printf("Error1: Argument must be between 0 and 7\n");
		return 0.0;
	}
	
	int jnt[8];
	
	jnt[0] = 8; // right knee
	jnt[1] = 6; // right hip roll
	jnt[2] = 3; // left hip roll
	jnt[3] = 1; // left knee
	jnt[4] = 9; // right ankle
	jnt[5] = 0; // left ankle
	jnt[6] = 4; // left pitch
	jnt[7] = 5; // right pitch
	MdtLimitID limitID = MdtHingeGetLimit(joints[jnt[i]]);	

	return	MdtLimitGetPosition(limitID);  
}


void Agent::SetJointMoveValue(int i, float Oj)
{
	// 0: left knee, 1: left pitch, 2: left roll 3: right roll 4: right pitch 5: right knee

	int jnt[8];

	jnt[0] = 8; // right knee
	jnt[1] = 6; // right hip roll
	jnt[2] = 3; // left hip roll
	jnt[3] = 1; // left knee
	jnt[4] = 9; // right ankle
	jnt[5] = 0; // left ankle
	jnt[6] = 4; // left pitch
	jnt[7] = 5; // right pitch

	switch (i)
	{
		case 0: // 
			move_value[0] = Oj*(PI/4 + PI/4 - 0.01);
			break;
		case 1: //pitch
			move_value[1] = Oj*PI/-5.1;
			break;
		case 2:
			move_value[2] = Oj*PI/5.1; // Minus eingefügt, da Extension, statt Flexion!!
			break;
		case 3:
			move_value[3] =Oj*(PI/4 + PI/4 - 0.01); // Minus eingefügt, da Extension, statt Flexion!!
			break;
		case 4: //pitch
			move_value[4] = -Oj*PI/5.1; 
			break;
		case 5: // 
			move_value[5] =  -Oj*PI/5.1; 
			break;
		case 6: // 
			move_value[6] = 0;	//statt 9.1 eine 3 eingesetzt!
			break;
		case 7: // 
			move_value[7] = 0;
			break;
	}
}

void Agent::SetJointMoveValue(int i, float Oj, float force)
{
	// 0: left knee, 1: left pitch, 2: left roll
	// 3: right roll 4: right pitch 5: right knee

	int jnt[8];

	jnt[0] = 8; // right knee
	jnt[1] = 6; // right hip roll
	jnt[2] = 3; // left hip roll
	jnt[3] = 1; // left knee
	jnt[4] = 9; // right ankle
	jnt[5] = 0; // left ankle
	jnt[6] = 4; // left pitch
	jnt[7] = 5; // right pitch

	switch (i)
	{
		case 0: // 
			move_value[0] = Oj*PI/4 + PI/4 - 0.01;
			break;
		case 1: //pitch
			move_value[1] = Oj*PI/-3.1;
			break;
		case 2:
			move_value[2] = Oj*PI/3.1; // Minus eingefügt, da Extension, statt Flexion!!
			break;
		case 3:
			move_value[3] = Oj*PI/4 + PI/4 - 0.01; // Minus eingefügt, da Extension, statt Flexion!!
			break;
		case 4: //pitch
			move_value[4] = -Oj*PI/3.1 + 0.1; 
			break;
		case 5: // 
			move_value[5] =  -Oj*PI/3.1 + 0.1; 
			break;
		case 6: // 
			move_value[6] = 0;	//statt 9.1 eine 3 eingesetzt!
			break;
		case 7: // 
			move_value[7] = 0;
			break;
	}
	forces[i] = force;
}


int Agent::GetFootContact(int i)
{
	if (i == 0)
		return lfc;
	else if (i == 1)
		return rfc;
	else
	{
		cerr << "Argument can only be O or 1 \n" << endl; 
		return 0;
	} 
}


float Agent::GetGolgiValues(int i) 
{// This is only an inaccurate approximation. It WILL NOT work when biped is on the ground.
	
	float oldVelocity, newVelocity;

	if ((i < 0) || (i > 11)) printf("Error2: i must be between 0 and 11");
	else
	{
		int jnt[8];

		jnt[0] = 8; // right knee
		jnt[1] = 6; // right hip roll
		jnt[2] = 3; // left hip roll
		jnt[3] = 1; // left knee
		jnt[4] = 9; // right ankle
		jnt[5] = 0; // left ankle
		jnt[6] = 4; // left pitch
		jnt[7] = 5; // right pitch
	
		int j;
		if (i > 5) j = i - 6;
		else j = i;

		switch(mode[j])
		{
			case 0: //position control
				printf("error!");
				return -1.0;
			case 1: //force control
				switch(i)
				{
					case 0: // left knee flex
						return move_value[j]/1.09; 
					case 1: // left hip flex 
						if (move_value[j] < -0.1)
							return move_value[j]/-8.63;
						else return 0.0;
					case 2: // right hip flex
						if (move_value[j] > 0.1)
							return move_value[j]/8.63;
						else return 0.0;
					case 3: // right knee flex
						return move_value[j]/1.09; 
					case 7: // left hip ext
						if (move_value[j] > 0.1) 
							return move_value[j]/8.63;
						else return 0.0;
					case 8: // right hip ext
						if (move_value[j] < -0.1)
							return move_value[j]/-8.63;
						else return 0.0;
				}
			case 2: // turning motors off
				return 0.0;
			case 3: // cocontraction
				return forces[j]/5.0;
			case 4: 
				if ((i == 0)  || (i == 3))	
					return 0.0; 
				else 
					return forces[j]/5.0;
		}
	}	
}

float Agent::GetMuscleDynamicSpindleValues(int i)//new dynamic case
{
	if ((i < 0) || (i > 11))
		printf("Error3: i must be between 0 and 5");
	else
	{
		int jnt[8];
	
		jnt[0] = 8; // left knee
		jnt[1] = 6; // left hip roll
		jnt[2] = 3; // right hip roll
		jnt[3] = 1; // right knee
		jnt[4] = 9; // right ankle	
		jnt[5] = 0; // left ankle	
		jnt[6] = 4; // left pitch		
		jnt[7] = 5; // right pitch

		int j;
		if (i > 5) j = i - 6;
		else j = i;	

		MdtLimitID limitID = MdtHingeGetLimit(joints[jnt[j]]);		
		float nvelocity = MdtLimitGetVelocity(limitID);
		float nposition = MdtLimitGetPosition(limitID);	

	
		if (mode[j]  == 2) 
			return 0.0; 
		else
		{ 
			if (nvelocity > 1)  // Extension
			{
				if (i == j) return 0.0; 
				else
				{ // return 0.8; 
					if ((j == 0) || (j == 3))
						return (nvelocity*0.20/3.36 + 0.2);
					if ((j == 2)|| (j==1)) 
						return (nvelocity*0.30/4.96 + 0.2);
					if ((j == 4) || (j == 5)) 
						return  (nvelocity*0.03/2.43 + 0.2);
				} 
			}
			else if (nvelocity < -1)
			{			//Flexion
				if (i == j)
				{ // return 0.8; 
					if ((j == 0) || (j == 3))
						return (nvelocity*0.30/-3.36 + 0.2);
					if ((j == 2)|| (j==1)) 
						return (nvelocity*0.40/-4.96 + 0.2);
					if ((j == 4) || (j == 5)) 
						return  (nvelocity*0.05/-2.43 + 0.2);//0.35
				}
				else return 0.0; 
			}
			else 
			{
				if (nposition > 0.1) 
				{
					if (i == j) return 0.0;
					else return 0.0; 
				}
				else if (nposition < -0.1) 
				{ 
					if (i == j) return 0.0; 
					else return 0.0; 
				}
				else if ((mode[j] == 3) || (mode[j] == 4))
				{
					if ((j == 0) || (j == 3)) 
					{ // knee extension
						if (i == j) return 0.0;
						else return 0.0;
					}
					else return 0.0;
				}
				else return 0.0;
			}
		}
	}	
}
float Agent::GetMuscleSpindleValues(int i)//static case
{
	float hipspindle, kneespindle, footspindle;
	if ((i < 0) || (i > 11))
		printf("Error3: i must be between 0 and 11");
	else
	{
		int jnt[8];
	
		jnt[0] = 8; // left knee
		jnt[1] = 6; // left hip roll
		jnt[2] = 3; // right hip roll
		jnt[3] = 1; // right knee
		jnt[4] = 9; // right ankle
		jnt[5] = 0; // left ankle
		jnt[6] = 4; // left pitch
		jnt[7] = 5; // right pitch

		int j;
		if (i > 5) j = i - 6;
		else j = i;

		MdtLimitID limitID = MdtHingeGetLimit(joints[jnt[j]]);	
		float nposition = MdtLimitGetPosition(limitID);
		
		if((j==1) || (j==2)) //Hip
		{
			if (j==2) 
				nposition = -nposition;
			hipspindle = 0.0;
			hipspindle = (nposition + HIP_ROLL_MAXIMUM)/(2*HIP_ROLL_MAXIMUM);
			if((i==7) || (i==8)) //extensor
				return pow(hipspindle, EXP)/5;
			else if ((i==1)||(i==2))   //flexor
				return pow(1-hipspindle, EXP)/5;
			else
				return 0.0;
		}
		else if ((j==0) || (j==3)) //knee
		{
			kneespindle = 0.0;
			kneespindle = (nposition - KNEE_MINIMUM)/(KNEE_MAXIMUM);
			if (kneespindle > 1)
				kneespindle = 1;
			else if (kneespindle <0)
				kneespindle = 0;
			if((i==6) || (i==9)) //extensor
				return pow(kneespindle, EXP)/5;
			else if ((i==0) || (i==3))
				return pow(1-kneespindle, EXP)/5;
			else
				return 0.0;
		}
		else if ((j==4) || (j==5)) //foot
		{
			footspindle = 0.0;
			footspindle = (nposition + ANKLE_MAXIMUM)/(2*ANKLE_MAXIMUM);
			if (footspindle > 1)
				footspindle = 1;
			else if (footspindle < 0)
				footspindle = 0;
			if((i==10) || (i==11))
				return pow(footspindle, EXP)/5;
			else if ((i==4) || (i==5))
				return pow(1-footspindle, EXP)/5;
			else
				return 0.0;
		}
	}
	hipspindle  = 0.0;
	kneespindle = 0.0;
	footspindle = 0.0;
}

float Agent::GetCutaneousValues(int i)
{
	MeVector3 currPos; 

	if (i == 0) 
		return lfc/2; 
	else if (i == 1) 
		return rfc/2;
	else
	{
		cerr << "Argument can only be O or 1 \n" << endl; 
		return 0;
	} 
}

float Agent::getJointAngle(int jointID)
{
	if ((jointID < 0) || (jointID > 11))
		printf("Error3: jointID must be between 0 and 5");
	else
	{
		int joint[6];
	
		joint[0] = 8; // left knee
		joint[1] = 6; // left hip roll
		joint[2] = 3; // right hip roll
		joint[3] = 1; // right knee
		joint[4] = 9; // right ankle
		joint[5] = 0; // left ankle

		int j;
		if (jointID > 5) j = jointID - 6;
		else j = jointID;

		MdtLimitID limitID = MdtHingeGetLimit(joints[joint[j]]);	
		float nposition = MdtLimitGetPosition(limitID);

		return nposition/PI*180;	//returns angle between body parts in degrees
	}
}


/* ---------------------------------------------------
							PRIVATE METHODS
 -------------------------------------------------- */

void Agent::ConstrainJoints(void)
{
	int leftHipPitch = 4;
	int rightHipPitch = 5;
	int leftHipRoll = 3;
	int rightHipRoll = 6;
	int leftKneeJoint = 1;
	int rightKneeJoint = 8;
	int leftAnkleJoint = 0;
	int rightAnkleJoint =9;

	SetJointLimits(leftHipPitch,HIP_PITCH_MINIMUM,HIP_PITCH_MAXIMUM);
	SetJointLimits(rightHipPitch,HIP_PITCH_MINIMUM,HIP_PITCH_MAXIMUM);

	SetJointLimits(leftHipRoll,HIP_ROLL_MINIMUM,HIP_ROLL_MAXIMUM);
	SetJointLimits(rightHipRoll,HIP_ROLL_MINIMUM,HIP_ROLL_MAXIMUM);

	SetJointLimits(leftKneeJoint,KNEE_MINIMUM,KNEE_MAXIMUM);
	SetJointLimits(rightKneeJoint,KNEE_MINIMUM,KNEE_MAXIMUM);

	SetJointLimits(leftAnkleJoint, ANKLE_MINIMUM,ANKLE_MAXIMUM);
	SetJointLimits(rightAnkleJoint,ANKLE_MINIMUM,ANKLE_MAXIMUM);

	// Knee fixed Joints
	WeldJoint(2);
	WeldJoint(7);

	// Hip pitch joints
	WeldJoint(4);
	WeldJoint(5);

	// ankle fixed Joints
	WeldJoint(10);
	WeldJoint(11);
}

void Agent::CreateBodyParts(void)
{
	bodyParts = new MdtBodyID[NUM_BODY_PARTS];
	bodyPartsCM = new McdModelID[NUM_CONNECTORS+2];
	bodyParts_prim = new McdGeometryID[NUM_CONNECTORS+2];
	bodyWaistCM = new McdModelID[NUM_CONNECTORS+4];
	bodyWaist_prim =  new McdGeometryID[NUM_CONNECTORS+4];
	limits = new MdtLimitID[NUM_JOINTS];

	CreateFeetKneesHips();
	CreateCylinders();
	CreateJoints();
	ConstrainJoints();
	CreateTarget();
}

void Agent::CreateCylinders(void)
{
	int currBodyPart;
	MeVector4 quat;
	MeVector3 ex, ey, ez;
	MeMatrix3 tempMatrix;

	ex[0] = 1.0;
	ex[1] = 0.0;
	ex[2] = 0.0;

	ey[0] = 0.0;
	ey[1] = 1.0;
	ey[2] = 0.0;

	ez[0] = 0.0;
	ez[1] = 0.0;
	ez[2] = 1.0;

	for (currBodyPart=NUM_CONNECTORS;currBodyPart<NUM_BODY_PARTS;currBodyPart++)
	{
		bodyParts[currBodyPart] = MdtBodyCreate(world);
		MdtBodySetAngularVelocity(bodyParts[currBodyPart], 0, 0, 0);
	}

	// Lower Legs
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS], -BODY_WIDTH/2.0, DEFAULT_RADIUS+DEF_LL_LENGTH/2.0 + HEIGHT_OFFSET , 0.0);
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+1], BODY_WIDTH/2.0, DEFAULT_RADIUS+DEF_LL_LENGTH/2.0 + HEIGHT_OFFSET, 0.0);

	// Upper Legs
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+2], -BODY_WIDTH/2.0, DEFAULT_RADIUS+DEF_LL_LENGTH+DEF_UL_LENGTH/2.0 + HEIGHT_OFFSET, 0.0);
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+3], BODY_WIDTH/2.0, DEFAULT_RADIUS+DEF_LL_LENGTH+DEF_UL_LENGTH/2.0 + HEIGHT_OFFSET, 0.0);

	// Waist
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+4], 0.0, DEFAULT_RADIUS+DEF_LL_LENGTH+DEF_UL_LENGTH + HEIGHT_OFFSET, 0.0);
	
	//Feet
    MdtBodySetPosition(bodyParts[NUM_CONNECTORS+5], -BODY_WIDTH/2.0, DEF_FOOT_THICKNESS/2 + HEIGHT_OFFSET, -DEF_FOOT_LENGTH/2.0);
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+6], BODY_WIDTH/2.0 , DEF_FOOT_THICKNESS/2 + HEIGHT_OFFSET, -DEF_FOOT_LENGTH/2.0);

	for (currBodyPart=NUM_CONNECTORS;currBodyPart<NUM_BODY_PARTS;currBodyPart++)
	{
		if ( currBodyPart < (NUM_CONNECTORS+4) )
		{
			MeQuaternionForRotation(quat,ez,ey);
			MdtBodySetQuaternion(bodyParts[currBodyPart],quat[0],quat[1],quat[2],quat[3]);
		}
		else
		{
			MeMatrix3MakeRotationY(PI/2.0,tempMatrix);
			MdtBodySetOrientation(bodyParts[currBodyPart],tempMatrix);
		}
	}

	// Lower leg stuff
	for (currBodyPart=NUM_CONNECTORS;currBodyPart<NUM_CONNECTORS+2;currBodyPart++)
	{
		MdtBodySetMass(bodyParts[currBodyPart],DEF_LL_MASS);
		bodyPartsG[currBodyPart] = RCreateCylinder(rc, DEF_LL_WIDTH, DEF_LL_LENGTH, green, MdtBodyGetTransformPtr(bodyParts[currBodyPart]));
	}

   // Upper leg stuff
	for (currBodyPart=NUM_CONNECTORS+2;currBodyPart<NUM_CONNECTORS+4;currBodyPart++)
	{
		MdtBodySetMass(bodyParts[currBodyPart],DEF_UL_MASS);
		bodyPartsG[currBodyPart] = RCreateCylinder(rc, DEF_UL_WIDTH, DEF_UL_LENGTH, green, MdtBodyGetTransformPtr(bodyParts[currBodyPart]));
	}

	// Waist stuff
	MdtBodySetMass(bodyParts[NUM_CONNECTORS+4],DEF_WAIST_MASS);
	bodyPartsG[NUM_CONNECTORS+4] = RCreateCylinder(rc, DEF_WAIST_WIDTH, BODY_WIDTH, green, MdtBodyGetTransformPtr(bodyParts[NUM_CONNECTORS+4]));

    //Foot Plate Stuff
	for (currBodyPart=NUM_CONNECTORS+5;currBodyPart<NUM_CONNECTORS+7;currBodyPart++)
	{
		MdtBodySetMass(bodyParts[currBodyPart],DEF_FOOT_MASS);
		bodyPartsG[currBodyPart] = RCreateCube(rc, DEF_FOOT_LENGTH, DEF_FOOT_THICKNESS, DEF_FOOT_WIDTH, green, MdtBodyGetTransformPtr(bodyParts[currBodyPart]));
		
		bodyParts_prim[currBodyPart-5] = McdBoxCreate(DEF_FOOT_LENGTH, DEF_FOOT_THICKNESS, DEF_FOOT_WIDTH);
		bodyPartsCM[currBodyPart-5] = McdModelCreate(bodyParts_prim[currBodyPart-5]);
		McdSpaceInsertModel(space, bodyPartsCM[currBodyPart-5]);
		McdDtBridgeSetBody(cdHandler, bodyPartsCM[currBodyPart-5], bodyParts[currBodyPart]);
		
		if (currBodyPart == NUM_CONNECTORS+5)
		{
			McdDtBridgeSetMaterialID(bodyPartsCM[currBodyPart-5], leftFootMaterial);
			McdDtBridgeSetContactCB(leftFootMaterial, floorMaterial, LeftFootContactCB); 
		}
		else if (currBodyPart == NUM_CONNECTORS+6)
		{
			McdDtBridgeSetMaterialID(bodyPartsCM[currBodyPart-5], rightFootMaterial);
			McdDtBridgeSetContactCB(rightFootMaterial, floorMaterial, RightFootContactCB);
		}
	}

	bodyWaist_prim[NUM_CONNECTORS+4] = McdCylinderCreate(DEF_WAIST_WIDTH,BODY_WIDTH);
	bodyWaistCM[NUM_CONNECTORS+4] = McdModelCreate(bodyWaist_prim[NUM_CONNECTORS+4]);
	McdSpaceInsertModel(space, bodyWaistCM[NUM_CONNECTORS+4]);
	McdDtBridgeSetBody(cdHandler, bodyWaistCM[NUM_CONNECTORS+4],bodyParts[NUM_CONNECTORS+4]);
	
	McdDtBridgeSetMaterialID(bodyWaistCM[NUM_CONNECTORS+4], waistMaterial);
	McdDtBridgeSetContactCB(waistMaterial,floorMaterial,WaistContactCB);//contact between waist and cube

	for (currBodyPart=NUM_CONNECTORS;currBodyPart<NUM_BODY_PARTS;currBodyPart++)
		MdtBodyEnable(bodyParts[currBodyPart]);
}

void Agent::CreateFeetKneesHips(void)
{
	int currBodyPart;
	MeMatrix3 I;

	for (currBodyPart=0;currBodyPart<NUM_CONNECTORS;currBodyPart++)
	{
		bodyParts[currBodyPart] = MdtBodyCreate(world);
		MdtBodyEnable(bodyParts[currBodyPart]);

		MdtBodySetMass(bodyParts[currBodyPart],DEF_CONNECTOR_MASS);
		MdtBodySetAngularVelocity(bodyParts[currBodyPart], 0, 0, 0);
		MdtMakeInertiaTensorSphere(DEF_CONNECTOR_MASS, DEFAULT_RADIUS,I);
		MdtBodySetInertiaTensor(bodyParts[currBodyPart],I);

		bodyPartsG[currBodyPart] = RCreateSphere(rc, DEFAULT_RADIUS, purple, MdtBodyGetTransformPtr(bodyParts[currBodyPart]));
		/* ball collision */
		bodyParts_prim[currBodyPart] = McdSphereCreate(DEFAULT_RADIUS);
		bodyPartsCM[currBodyPart] = McdModelCreate(bodyParts_prim[currBodyPart]);
		McdSpaceInsertModel(space, bodyPartsCM[currBodyPart]);
		McdDtBridgeSetBody(cdHandler, bodyPartsCM[currBodyPart], bodyParts[currBodyPart]);
		
		if (currBodyPart == 0)
			McdDtBridgeSetMaterialID(bodyPartsCM[currBodyPart], leftFootMaterial);
		if (currBodyPart == 1)
			McdDtBridgeSetMaterialID(bodyPartsCM[currBodyPart], rightFootMaterial);
	}

	//Positions
	// Feet Connectors
	MdtBodySetPosition(bodyParts[0],-BODY_WIDTH/2,DEFAULT_RADIUS + HEIGHT_OFFSET,0.0);
	MdtBodySetPosition(bodyParts[1],BODY_WIDTH/2,DEFAULT_RADIUS + HEIGHT_OFFSET,0.0);

	// Knee Connectors
	MdtBodySetPosition(bodyParts[2],-BODY_WIDTH/2,DEFAULT_RADIUS+DEF_LL_LENGTH+HEIGHT_OFFSET,0.0);
	MdtBodySetPosition(bodyParts[3],BODY_WIDTH/2,DEFAULT_RADIUS+DEF_LL_LENGTH+HEIGHT_OFFSET,0.0);

	// Hip Connectors
	MdtBodySetPosition(bodyParts[4],-BODY_WIDTH/2,DEFAULT_RADIUS+DEF_LL_LENGTH+DEF_UL_LENGTH+HEIGHT_OFFSET,0.0);
	MdtBodySetPosition(bodyParts[5],BODY_WIDTH/2,DEFAULT_RADIUS+DEF_LL_LENGTH+DEF_UL_LENGTH+HEIGHT_OFFSET,0.0);
}

void Agent::CreateJoints(void)
{
	int jointNum;
	MeVector3 connPos;

	// Left foot moving joint
	joints[0] = MdtHingeCreate(world,bodyParts[0],bodyParts[6]);
	MdtBodyGetPosition(bodyParts[0],connPos);
	MdtHingeSetPosition(joints[0],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[0],-1.0,0.0,0.0);
	MdtHingeEnable(joints[0]);

	// Left knee moving joint
	joints[1] = MdtHingeCreate(world,bodyParts[6],bodyParts[8]);
	MdtBodyGetPosition(bodyParts[2],connPos);
	MdtHingeSetPosition(joints[1],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[1],-1.0,0.0,0.0);
	MdtHingeEnable(joints[1]);

	// Left knee fixed joint
	joints[2] = MdtHingeCreate(world,bodyParts[2],bodyParts[8]);
	MdtBodyGetPosition(bodyParts[2],connPos);
	MdtHingeSetPosition(joints[2],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[2],1.0,0.0,0.0);
	MdtHingeEnable(joints[2]);

	// Left roll
	joints[3] = MdtHingeCreate(world,bodyParts[8],bodyParts[4]);
	MdtBodyGetPosition(bodyParts[4],connPos);
	MdtHingeSetPosition(joints[3],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[3],1.0,0.0,0.0);
	MdtHingeEnable(joints[3]);

	// Left pitch
	joints[4] = MdtHingeCreate(world,bodyParts[4],bodyParts[10]);
	MdtBodyGetPosition(bodyParts[4],connPos);
	MdtHingeSetPosition(joints[4],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[4],0.0,1.0,0.0);
	MdtHingeEnable(joints[4]);

	// Right pitch
	joints[5] = MdtHingeCreate(world,bodyParts[5],bodyParts[10]);
	MdtBodyGetPosition(bodyParts[5],connPos);
	MdtHingeSetPosition(joints[5],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[5],0.0,1.0,0.0);
	MdtHingeEnable(joints[5]);

	// Right roll
	joints[6] = MdtHingeCreate(world,bodyParts[5],bodyParts[9]);
	MdtBodyGetPosition(bodyParts[5],connPos);
	MdtHingeSetPosition(joints[6],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[6],1.0,0.0,0.0);
	MdtHingeEnable(joints[6]);

	// Right knee fixed joint
	joints[7] = MdtHingeCreate(world,bodyParts[9],bodyParts[3]);
	MdtBodyGetPosition(bodyParts[3],connPos);
	MdtHingeSetPosition(joints[7],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[7],1.0,0.0,0.0);
	MdtHingeEnable(joints[7]);

	// Right knee moving joint
	joints[8] = MdtHingeCreate(world,bodyParts[9],bodyParts[7]);
	MdtBodyGetPosition(bodyParts[3],connPos);
	MdtHingeSetPosition(joints[8],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[8],1.0,0.0,0.0);
	MdtHingeEnable(joints[8]);

	// Right foot moving joint
	joints[9] = MdtHingeCreate(world,bodyParts[7],bodyParts[1]);
	MdtBodyGetPosition(bodyParts[1],connPos);
	MdtHingeSetPosition(joints[9],connPos[0],connPos[1],connPos[2]);
	MdtHingeSetAxis(joints[9],1.0,0.0,0.0);
	MdtHingeEnable(joints[9]);

    // Right foot plate fixed joint
	joints[10] = MdtHingeCreate(world,bodyParts[12],bodyParts[1]);
	MdtBodyGetPosition(bodyParts[1],connPos);
	MdtHingeSetPosition(joints[10],connPos[0],connPos[1]-DEFAULT_RADIUS,connPos[2]);
	MdtHingeSetAxis(joints[10],1.0,0.0,0.0);
	MdtHingeEnable(joints[10]);


	// Left foot plate fixed joint
	joints[11] = MdtHingeCreate(world,bodyParts[11],bodyParts[0]);
	MdtBodyGetPosition(bodyParts[0],connPos);
	MdtHingeSetPosition(joints[11],connPos[0],connPos[1]-DEFAULT_RADIUS,connPos[2]);
	MdtHingeSetAxis(joints[11],1.0,0.0,0.0);
	MdtHingeEnable(joints[11]);
}

void Agent::CreateTarget(void) {

	MeVector3 connPos;
	float TARGET_HEIGHT = DEF_FOOT_THICKNESS/2 + DEF_UL_LENGTH + DEF_LL_LENGTH+HEIGHT_OFFSET;
	float TARGET_WIDTH = TARGET_SIZE*5;
	float TARGET_LENGTH = TARGET_SIZE*30;
	
	target = MdtBodyCreate(world);
	MdtBodySetMass(target, 50.0);

	MdtBodySetPosition(target, 0.0, TARGET_HEIGHT/2, 0.0);
    targetG = RCreateCube(rc, TARGET_WIDTH, TARGET_HEIGHT, TARGET_LENGTH, red, MdtBodyGetTransformPtr(target));
    target_prim = McdBoxCreate(TARGET_WIDTH, TARGET_HEIGHT,TARGET_LENGTH);

	targetCM = McdModelCreate(target_prim);
    McdModelSetUserData(targetCM, NULL);
	McdDtBridgeSetBody(cdHandler, targetCM, target);
    McdSpaceInsertModel(space, targetCM); 
	MdtBodyEnable(target);
	
	joints[12] = MdtHingeCreate(world, target, bodyParts[10]);	
	MdtBodyGetPosition(target, connPos);
	MdtHingeSetPosition(joints[12],connPos[0],connPos[1]+ TARGET_HEIGHT/2,connPos[2]);
	MdtHingeSetAxis(joints[12],1.0,0.0,0.0);
	MdtHingeEnable(joints[12]);
	
	joints[13] = MdtHingeCreate(world, target, 0);
	MdtBodyGetPosition(target, connPos);
	MdtHingeSetPosition(joints[13], connPos[0], connPos[1] - TARGET_HEIGHT/2, connPos[2]);
	MdtHingeSetAxis(joints[13], 0.0, 0.0, 1.0);
	MdtHingeEnable(joints[13]);

	WeldJoint(12);
	WeldJoint(13);
}


void Agent::DestroyBodyParts(void)
{
	DestroyJoints();
	DestroyCylinders();
	DestroyFeetKneesHips();
	DestroyTarget();

	delete[] bodyParts;
	delete[] bodyPartsCM;
	delete[] bodyParts_prim;
}

void Agent::DestroyCylinders(void)
{
	int currBodyPart;

	for (currBodyPart=NUM_CONNECTORS;currBodyPart<NUM_BODY_PARTS;currBodyPart++)
	{
		RDeleteGraphic(bodyPartsG[currBodyPart]);
		MdtBodyDisable(bodyParts[currBodyPart]);
		MdtBodyDestroy(bodyParts[currBodyPart]);
	}
}

void Agent::DestroyFeetKneesHips(void)
{
	int currBodyPart;

	for (currBodyPart=0;currBodyPart<NUM_CONNECTORS+2;currBodyPart++)
	{
	    McdModelRemoveFromSpace(bodyPartsCM[currBodyPart]);
		McdModelDestroy(bodyPartsCM[currBodyPart]);

		McdGeometryDestroy(bodyParts_prim[currBodyPart]);

		if (currBodyPart < NUM_CONNECTORS)
		{
			RDeleteGraphic(bodyPartsG[currBodyPart]);

			MdtBodyDisable(bodyParts[currBodyPart]);
			MdtBodyDestroy(bodyParts[currBodyPart]);
		}
	}
}

void Agent::DestroyJoints(void)
{
	int jointNum;

	for ( jointNum=0;jointNum<(NUM_JOINTS+2);jointNum++)
	{
		MdtHingeDisable(joints[jointNum]);
		MdtHingeDestroy(joints[jointNum]);
	}
}

void Agent::DestroyTarget(void) {

	McdModelRemoveFromSpace(targetCM);
	McdModelDestroy(targetCM);
	McdGeometryDestroy(target_prim);

   	RDeleteGraphic(targetG);

	MdtBodyDisable(target);
	MdtBodyDestroy(target);
}


void Agent::SetJointLimits(int jointIndex, float limMin, float limMax)
{
	limits[jointIndex] = MdtHingeGetLimit(joints[jointIndex]);
	
	MdtSingleLimitSetStop(MdtLimitGetLowerLimit(limits[jointIndex]),limMin);
	MdtSingleLimitSetStop(MdtLimitGetUpperLimit(limits[jointIndex]),limMax); 
	MdtSingleLimitSetRestitution (MdtLimitGetLowerLimit(limits[jointIndex]), 0.2); //0.02
	MdtSingleLimitSetRestitution (MdtLimitGetUpperLimit(limits[jointIndex]), 0.2); //0.02

	MdtLimitActivateLimits(limits[jointIndex], 1);
}


void Agent::WeldJoint(int jointIndex)
{
	SetJointLimits(jointIndex,-0.01,0.01);
}


#endif