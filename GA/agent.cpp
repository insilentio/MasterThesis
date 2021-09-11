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

extern SIM_PARAMS *simParams;

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
extern int			LeftFootMaterial, RightFootMaterial, floorMaterial, waistMaterial;

extern float		HEIGHT_OFFSET;
extern float		TARGET_SIZE;
extern float		TARGET_DIST;

extern int			GA_FITNESS_ACTIVATE;
extern int			MAXRUNS;
extern int			DATACOUNT;
extern int			DIRECTIONBUFFER;

extern float	MarkPoint1;
extern float	MarkPoint2;
extern int      EXP;
int nextstep, a;

/* ---------------------------------------------------
							CALLBACK FUCNTIONS
 -------------------------------------------------- */

unsigned int LeftFootContactCB(MdtContactID mdtcontact,
									McdContact *mcdcontact,
									McdIntersectResult *result)
{
	
	lfootcontact = 1; 
	return 1; 
}


unsigned int RightFootContactCB(MdtContactID mdtcontact,
									McdContact *mcdcontact,
									McdIntersectResult *result)
{
	rfootcontact = 1; 
	return 1; 	
}

/*---------------------------------------------------------------
							Kollision zwischen Roboter und Würfel
---------------------------------------------------------------*/
unsigned int WaistContactCB(MdtContactID mdtcontact,
									McdContact *mcdcontact,
									McdIntersectResult *result)
{
	waistcontact = 1; 
	return 1; 	
}

/* ---------------------------------------------------
							PUBLIC METHODS
 -------------------------------------------------- */


AGENT::AGENT(void)
{
	printf("Creating an agent.\n");
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
	runs=0;
	fitness = 0;
	left_foot_distance  = 0;
	right_foot_distance = 0;
	left_step_distance = 0;
	right_step_distance = 0;
	muscleactivity = 0;
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

AGENT::~AGENT(void)
{
	//printf("Destroying an agent.\n");
	delete[] move_value;

	DestroyBodyParts(); 
}

void AGENT::Move(int g)
{
	

	MdtBodyGetPosition(bodyParts[0], pos_vector_foot_left_new);// get position of the both feet
	MdtBodyGetPosition(bodyParts[1], pos_vector_foot_right_new);
//	cout<<"linkes Bein in y-Richtung: "<<pos_vector_foot_left_new[2]<<endl;
//	cout<<"rechtes Bein in y-Richtung: "<<pos_vector_foot_right_new[2]<<endl;

	if ((g<17)) //needs several steps untill it starts moving the legs
	{
		PrepareWalkingDistanceCalculation();
	}
	else
	{
/////////////////////////////////////////////////////////////////////////////////////
// Controll if the foot is moving forwards or backwards

		FeetForwardsOrBackwards();

////////////////////////////////////////////////////////////////////////////////////////////
//check if the foot is changing the movement direction and set the max pos of the foot
		
		FeetChangingDirection();
		
///////////////////////////////////////////////////////////////////////////////////////////////
//if foot changed direction and max elongation position is known, calculate the stepdistance (and the total distance travelled of each foot) and find out if feet are oszillating
		if ((left_M1)&&(left_M2))
		{
			LeftFootStep();
//			cout<<"left Step at: "<<g<<endl;
		}
		
		else if ((right_M1)&&(right_M2))
		{
			RightFootStep();
//			cout<<"right Step at: "<<g<<endl;
		}

//////////////////////////////////////////////////////////////////////////////////////////////////
//to make sure, that after the buffertime, the legs are moving in opposing directions
		if (left_FORWARD_new==right_FORWARD_new)
			DirectionControll();
		else if (directionChange)//&&(!(left_FORWARD_new==right_FORWARD_new))) //one leg made two ch of dir more, they are still moving in opp dir.
		{
			directionBuffer = 0;
			directionChange = false;
			if (g>150)
			{
				if(AngleDiffCompare())
					HipAngleDiff = false;
				else
					HipAngleDiff =true;
			}
		}


		left_FORWARD_old = left_FORWARD_new;
		right_FORWARD_old = right_FORWARD_new;
			
	}

	
//	int nextStep;


//-------------------------------------------------------------------------------------------
// run is over; Fitness calculatoin

	if((SWINGING_LEG)&&(GA_FITNESS_ACTIVATE)) //to avoid that it just swings one leg 
	{
		fitness = right_foot_distance + left_foot_distance;
		simParams->evalOver = true;
		cout<<"SWINGING_LEG"<<runs<<" ; "<<right_foot_distance + left_foot_distance<<" ; "<<muscleactivity<<endl; 
		runs = 0;
		MuscleactivityReset();
		StepDistanceReset();
	}

	else if ((runs >700)&&(right_foot_distance == 0)&&(left_foot_distance == 0)&&(GA_FITNESS_ACTIVATE))
	{
		fitness = right_foot_distance + left_foot_distance;
		simParams->evalOver = true;
		cout<<"no step"<<runs<<" ; "<<right_foot_distance + left_foot_distance<<" ; "<<muscleactivity<<endl; 
		runs = 0;
		MuscleactivityReset();
		StepDistanceReset();
	}

	else if ((runs == MAXRUNS)&&(GA_FITNESS_ACTIVATE)) 
	{
		fitness = right_foot_distance + left_foot_distance;
		simParams->evalOver = true;
		cout<<"MAXRUNS"<<runs<<" ; "<<right_foot_distance + left_foot_distance<<" ; "<<muscleactivity<<endl; 
		runs = 0;
		MuscleactivityReset();
		StepDistanceReset();
	}

		else if((outOfPhase)&&(GA_FITNESS_ACTIVATE))
	{
		fitness = right_foot_distance + left_foot_distance;
		simParams->evalOver = true;
		cout<<"outOfPhase"<<runs<<" ; "<<right_foot_distance + left_foot_distance<<" ; "<<muscleactivity<<endl; 
		runs = 0;
		MuscleactivityReset();
		StepDistanceReset();
	}

		else if((HipAngleDiff)&&(GA_FITNESS_ACTIVATE))
	{
		fitness = right_foot_distance + left_foot_distance;
		simParams->evalOver = true;
		cout<<"Angle"<<runs<<" ; "<<right_foot_distance + left_foot_distance<<" ; "<<muscleactivity<<endl; 
		runs = 0;
		MuscleactivityReset();
		StepDistanceReset();
	}
//---------------------------
// run is not over jet
	else
	{
		runs++;
	//old version with 6 input values
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
}

void AGENT::TurnOffMotor(int currHingeJoint)
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

void AGENT::TurnOnMotor(int currHingeJoint)
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

float AGENT::GetJointPosition(int i)
{
	
	//0: left knee 1: left roll 2: right roll
	//3: right knee 4: left ankle 5: right ankle

	if ((i < 0) || (i > 5))
	{
		printf("Error1: Argument must be between 0 and 7\n");
		return 0.0;
	}
	
//old version with 6 input values
  
	int jnt[8];

	
	jnt[0] = 8; // right knee
	jnt[1] = 6; // right hip roll
	jnt[2] = 3; // left hip roll
	jnt[3] = 1; // left knee
	jnt[4] = 9; // right ankle
	jnt[5] = 0; // left ankle
	jnt[6] = 4; // left pitch
	jnt[7] = 5; // right pitch
/*
//new version with 12 input values	
	int jnt[12];
	
	jnt[0] = 1; //left knee flex
	jnt[1] = 1; //left knee ext
	jnt[2] = 3; //left hip flex
	jnt[3] = 3; //left hip ext
	jnt[4] = 6; //right hip flex
	jnt[5] = 6; //right hip ext
	jnt[6] = 8; //right knee flex
	jnt[7] = 8; //right knee ext
	jnt[8] = 0; //left ankle flex
	jnt[9] = 0; //left ankle ext
	jnt[10] = 9; //right ankle flex
	jnt[11] = 9; //right ankle ext
*/



	MdtLimitID limitID = MdtHingeGetLimit(joints[jnt[i]]);	

	return	MdtLimitGetPosition(limitID);  

}


void AGENT::SetJointMoveValue(int i, float Oj)
{
// 0: left knee, 1: left pitch, 2: left roll
	// 3: right roll 4: right pitch 5: right knee

	int jnt[8];
/*
	jnt[0] = 1; //left knee
	jnt[1] = 4; //left pitch
	jnt[2] = 3; //left roll
	jnt[3] = 6; //right roll
	jnt[4] = 5; // right pitch
	jnt[5] = 8; // right knee
	jnt[6] = 0; // left ankle
	jnt[7] = 9; // right ankle */


	jnt[0] = 8; // right knee
	jnt[1] = 6; // right hip roll
	jnt[2] = 3; // left hip roll
	jnt[3] = 1; // left knee
	jnt[4] = 9; // right ankle
	jnt[5] = 0; // left ankle
	jnt[6] = 4; // left pitch
	jnt[7] = 5; // right pitch

/*	switch (i)
	{
		case 0: // 
			position[0] = Oj*PI/4 + PI/4 - 0.01;
			break;
		case 1: //pitch
			position[1] = 0;
			break;
		case 2:
			position[2] = Oj*PI/7.1; // Minus eingefügt, da Extension, statt Flexion!!
			break;
		case 3:
			position[3] = Oj*PI/-7.1; // Minus eingefügt, da Extension, statt Flexion!!
			break;
		case 4: //pitch
			position[4] = 0;
			break;
		case 5: // 
			position[5] = Oj*PI/4 + PI/4 - 0.01;
			break;
		case 6: // 
			position[6] = -Oj*PI/3;	//statt 9.1 eine 3 eingesetzt!
			break;
		case 7: // 
			position[7] = Oj*PI/3;
			break;
	} */

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

void AGENT::SetJointMoveValue(int i, float Oj, float force)
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


int AGENT::GetFootContact(int i)
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


float AGENT::GetFitness()
{
	return fitness;
}


float AGENT::GetGolgiValues(int i) 
{
// This is only an inaccurate approximation. 
// It WILL NOT work when biped is on the ground. 
	
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


/*	oldVelocity = velocity[i];
	newVelocity = MdtLimitGetVelocity(limitID);

	float temp = newVelocity - oldVelocity; 
	velocity[i] = newVelocity;
	
	// F = MA
	if ((i == 4) || (i == 5)) // ankle
		return 6.4 * temp * 10/1000;
	else if ((i == 0) || (i == 3)) // knee
		return 26.4 * temp * 10/1000;
	else if ((i == 1) || (i == 2)) // roll
		return 62.4 * temp * 10/1000; */

	}	
}

float AGENT::GetMuscleDynamicSpindleValues(int i)//new dynamic case
{
		if ((i < 0) || (i > 11))
	{
		printf("Error3: i must be between 0 and 5");
	}
	
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
float AGENT::GetMuscleSpindleValues(int i)//static case
{
	float hipspindle, kneespindle, footspindle;
	if ((i < 0) || (i > 11))
	{
		printf("Error3: i must be between 0 and 11");
	}
	
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
//		float nvelocity = MdtLimitGetVelocity(limitID);
		float nposition = MdtLimitGetPosition(limitID);
		if((j==1)||(j==2)) //Hip
		{
			
//			if (j==1) cout<<nposition<<" "<<"left"<<j<<endl;
			if (j==2) 
			{
//				cout<<nposition<<" "<<"right"<<j<<endl;
				nposition=-nposition;
//				cout<<nposition<<" "<<"right"<<j<<endl;
			}
			hipspindle = 0.0;
			hipspindle = (nposition + HIP_ROLL_MAXIMUM)/(2*HIP_ROLL_MAXIMUM);
				if((i==7)||(i==8)) //extensor
				{
//					cout<<pow(hipspindle,EXP)/5<<" "<<"extensor"<<" "<<j<<endl;
					return pow(hipspindle,EXP)/5;
				}
				else if ((i==1)||(i==2))   //flexor
				{
//					cout<<pow(1-hipspindle,EXP)/5<<" "<<"flexor"<<" "<<j<<endl;
					return pow(1-hipspindle,EXP)/5;
				}
				else
					return 0.0;
		}
		else if ((j==0)||(j==3)) //knee
		{
			kneespindle = 0.0;
			kneespindle = (nposition - KNEE_MINIMUM)/(KNEE_MAXIMUM);
			if (kneespindle > 1)
				kneespindle = 1;
			else if (kneespindle <0)
				kneespindle = 0;
			if((i==6)||(i==9)) //extensor
				return pow(kneespindle,EXP)/5;
			else if ((i==0)||(i==3))
				return pow(1-kneespindle,EXP)/5;
			else
				return 0.0;
		}
		else if ((j==4)||(j==5)) //foot
		{
			footspindle = 0.0;
			footspindle = (nposition + ANKLE_MAXIMUM)/(2*ANKLE_MAXIMUM);
			if (footspindle >1)
				footspindle = 1;
			else if (footspindle < 0)
				footspindle = 0;
			if((i==10)||(i==11))
				return pow(footspindle,EXP)/5;
			else if ((i==4)||(i==5))
				return pow(1-footspindle,EXP)/5;
			else
				return 0.0;
		}
	}
	hipspindle = 0.0;
	kneespindle = 0.0;
	footspindle = 0.0;
}

float AGENT::GetCutaneousValues(int i) {
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

/*
	if ((i != 0) && (i != 1)) printf("Error4: i must be between 0 and 1\n");

	else {
		MdtBodyGetPosition(bodyParts[11+i], currPos);
		if (currPos[2] < 3*DEFAULT_RADIUS) return 0.5;
			else return 0.0;
	
	} */
}

float AGENT::getJointAngle(int jointID)
{
	if ((jointID < 0) || (jointID > 11))
	{
		printf("Error3: jointID must be between 0 and 5");
	}
	
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
		//cout<<"Winkel im Gelenk "<<joint[j]<<" ist "<<nposition<<endl;

		return nposition/PI*180;	//returns angle between body parts in degrees
	}
}

//--------------------------------------------------------------------------------------
//function to write angle and mn-activity into a txt-file
//--------------------------------------------------------------------------------------

void AGENT::WritetoFile( int ID, float channelValues[])
{
	char outFileName[50];
	int g;

	sprintf(outFileName,"data_txt/%d.txt",ID);

	outFile.open(outFileName);

	for ( g=0; g<DATACOUNT; g++ )
		outFile << channelValues[g] << "\n";

	outFile.close();
}


/* ---------------------------------------------------
							PRIVATE METHODS
 -------------------------------------------------- */


void AGENT::ConstrainJoints(void)
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

void AGENT::CreateBodyParts(void)
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

void AGENT::CreateCylinders(void)
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
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS],
							 -BODY_WIDTH/2.0,
							 DEFAULT_RADIUS+DEF_LL_LENGTH/2.0 + HEIGHT_OFFSET ,
							 0.0);
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+1],
							 BODY_WIDTH/2.0,
							 DEFAULT_RADIUS+DEF_LL_LENGTH/2.0 + HEIGHT_OFFSET,
							 0.0);

	// Upper Legs
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+2],
							 -BODY_WIDTH/2.0,
							 DEFAULT_RADIUS+DEF_LL_LENGTH+DEF_UL_LENGTH/2.0 + HEIGHT_OFFSET,
							 0.0);
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+3],
							 BODY_WIDTH/2.0,
							 DEFAULT_RADIUS+DEF_LL_LENGTH+DEF_UL_LENGTH/2.0 + HEIGHT_OFFSET,
							 0.0);

	// Waist
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+4],
							 0.0,
							 DEFAULT_RADIUS+DEF_LL_LENGTH+DEF_UL_LENGTH + HEIGHT_OFFSET,
							 0.0);
	
	//Feet
    MdtBodySetPosition(bodyParts[NUM_CONNECTORS+5],
							 -BODY_WIDTH/2.0,
							  DEF_FOOT_THICKNESS/2 + HEIGHT_OFFSET,
					    	 -DEF_FOOT_LENGTH/2.0);
	MdtBodySetPosition(bodyParts[NUM_CONNECTORS+6],
							 BODY_WIDTH/2.0 ,
							 DEF_FOOT_THICKNESS/2 + HEIGHT_OFFSET,
							 -DEF_FOOT_LENGTH/2.0);

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
		bodyPartsG[currBodyPart] = RCreateCylinder(rc,
										DEF_LL_WIDTH,
										DEF_LL_LENGTH,
										green,
										MdtBodyGetTransformPtr(bodyParts[currBodyPart]));
	}

   // Upper leg stuff
	for (currBodyPart=NUM_CONNECTORS+2;currBodyPart<NUM_CONNECTORS+4;currBodyPart++)
	{
		MdtBodySetMass(bodyParts[currBodyPart],DEF_UL_MASS);
		bodyPartsG[currBodyPart] = RCreateCylinder(rc,
										DEF_UL_WIDTH,
										DEF_UL_LENGTH,
										green,
										MdtBodyGetTransformPtr(bodyParts[currBodyPart]));
	}

	// Waist stuff
	MdtBodySetMass(bodyParts[NUM_CONNECTORS+4],DEF_WAIST_MASS);
	bodyPartsG[NUM_CONNECTORS+4] = RCreateCylinder(rc,
										DEF_WAIST_WIDTH,
										BODY_WIDTH,
										green,
										MdtBodyGetTransformPtr(bodyParts[NUM_CONNECTORS+4]));

    //Foot Plate Stuff
	for (currBodyPart=NUM_CONNECTORS+5;currBodyPart<NUM_CONNECTORS+7;currBodyPart++)
	{
		MdtBodySetMass(bodyParts[currBodyPart],DEF_FOOT_MASS);
		bodyPartsG[currBodyPart] = RCreateCube(rc,
										DEF_FOOT_LENGTH,
										DEF_FOOT_THICKNESS,
										DEF_FOOT_WIDTH,
										green,
										MdtBodyGetTransformPtr(bodyParts[currBodyPart]));
		
		bodyParts_prim[currBodyPart-5] = McdBoxCreate(DEF_FOOT_LENGTH, DEF_FOOT_THICKNESS, DEF_FOOT_WIDTH);
		bodyPartsCM[currBodyPart-5] = McdModelCreate(bodyParts_prim[currBodyPart-5]);
		McdSpaceInsertModel(space, bodyPartsCM[currBodyPart-5]);
		McdDtBridgeSetBody(cdHandler, bodyPartsCM[currBodyPart-5], bodyParts[currBodyPart]);
		
		if (currBodyPart == NUM_CONNECTORS+5)
		{
			McdDtBridgeSetMaterialID(bodyPartsCM[currBodyPart-5], LeftFootMaterial);
			McdDtBridgeSetContactCB(LeftFootMaterial, floorMaterial, LeftFootContactCB); 
		}

		if (currBodyPart == NUM_CONNECTORS+6)
		{
			McdDtBridgeSetMaterialID(bodyPartsCM[currBodyPart-5], RightFootMaterial);
			McdDtBridgeSetContactCB(RightFootMaterial, floorMaterial, RightFootContactCB);
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

void AGENT::CreateFeetKneesHips(void)
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
		{
			McdDtBridgeSetMaterialID(bodyPartsCM[currBodyPart], LeftFootMaterial);
	//		McdDtBridgeSetContactCB(LeftFootMaterial, floorMaterial, LeftFootContactCB); 
		}

		if (currBodyPart == 1)
		{
			McdDtBridgeSetMaterialID(bodyPartsCM[currBodyPart], RightFootMaterial);
	//		McdDtBridgeSetContactCB(RightFootMaterial, floorMaterial, RightFootContactCB);
		}
		
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

void AGENT::CreateJoints(void)
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

void AGENT::CreateTarget(void) {

	MeVector3 connPos;
	float TARGET_HEIGHT = DEF_FOOT_THICKNESS/2 + DEF_UL_LENGTH + DEF_LL_LENGTH+HEIGHT_OFFSET;
	float TARGET_WIDTH = TARGET_SIZE*5;
	float TARGET_LENGTH = TARGET_SIZE*30;
	
	target = MdtBodyCreate(world);
	MdtBodySetMass(target, 50.0);

	MdtBodySetPosition(target, 0.0, TARGET_HEIGHT/2, 0.0);
    targetG = RCreateCube(rc, TARGET_WIDTH, TARGET_HEIGHT, TARGET_LENGTH, red, 
		                  MdtBodyGetTransformPtr(target));
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


void AGENT::DestroyBodyParts(void)
{
//	delete[] limits;

	DestroyJoints();
	DestroyCylinders();
	DestroyFeetKneesHips();
	DestroyTarget();

	delete[] bodyParts;
	delete[] bodyPartsCM;
	delete[] bodyParts_prim;
}

void AGENT::DestroyCylinders(void)
{
	int currBodyPart;

	for (currBodyPart=NUM_CONNECTORS;currBodyPart<NUM_BODY_PARTS;currBodyPart++)
	{
		RDeleteGraphic(bodyPartsG[currBodyPart]);

		MdtBodyDisable(bodyParts[currBodyPart]);
		MdtBodyDestroy(bodyParts[currBodyPart]);
	}
}

void AGENT::DestroyFeetKneesHips(void)
{
	int currBodyPart;

	for (currBodyPart=0;currBodyPart<NUM_CONNECTORS+2;currBodyPart++)
	{
	    McdModelRemoveFromSpace(bodyPartsCM[currBodyPart]);
		McdModelDestroy(bodyPartsCM[currBodyPart]);

		McdGeometryDestroy(bodyParts_prim[currBodyPart]);

		if (currBodyPart < NUM_CONNECTORS) {
		RDeleteGraphic(bodyPartsG[currBodyPart]);

		MdtBodyDisable(bodyParts[currBodyPart]);
		MdtBodyDestroy(bodyParts[currBodyPart]);
		}
	}
}

void AGENT::DestroyJoints(void)
{
	int jointNum;

	for ( jointNum=0;jointNum<(NUM_JOINTS+2);jointNum++)
	{
		MdtHingeDisable(joints[jointNum]);
		MdtHingeDestroy(joints[jointNum]);
	}
}

void AGENT::DestroyTarget(void) {

	McdModelRemoveFromSpace(targetCM);
	McdModelDestroy(targetCM);
	McdGeometryDestroy(target_prim);

   	RDeleteGraphic(targetG);

	MdtBodyDisable(target);
	MdtBodyDestroy(target);

}


void AGENT::SetJointLimits(int jointIndex, float limMin, float limMax)
{
	limits[jointIndex] = MdtHingeGetLimit(joints[jointIndex]);
	
	MdtSingleLimitSetStop(MdtLimitGetLowerLimit(limits[jointIndex]),limMin);
	MdtSingleLimitSetStop(MdtLimitGetUpperLimit(limits[jointIndex]),limMax); 
	MdtSingleLimitSetRestitution (MdtLimitGetLowerLimit(limits[jointIndex]), 0.2); //0.02
	MdtSingleLimitSetRestitution (MdtLimitGetUpperLimit(limits[jointIndex]), 0.2); //0.02


	MdtLimitActivateLimits(limits[jointIndex], 1);
}


void AGENT::WeldJoint(int jointIndex)
{
	SetJointLimits(jointIndex,-0.01,0.01);
}

void AGENT::MuscleactivityCalculate(float i)
{
	muscleactivity += i;
}


void AGENT::MuscleactivityReset()
{
	muscleactivity = 0.0;
}

void AGENT::PrepareWalkingDistanceCalculation()
{
	SettingVariables();
}
//////////////////////////////////////////////////////////////////////////
//setting al variables used in to calculate the fitness (walking distance)
void AGENT::SettingVariables()
{
	SettingLeftFootVariables();
	
	SettingRightFootVariables();
	
	SWINGING_LEG = false; //to make sure no leg takes twice a step
	directionChange_left = false;
	directionChange_right = false;
	directionBuffer=0;
	outOfPhase=false;	// to make sure the legs are oszillating in opposit direction
	HipAngleDiff = false;
	directionChange = false;
}

void AGENT::SettingLeftFootVariables()
{
	max_fw_pos_left= 0.0;
	max_bw_pos_left= 0.0;
	max_fw_angle_left = 0.0;
	max_bw_angle_left = 0.0;
	angle_difference_left = 0;
	leftStep = 0; //Zähler der Schritte (li / re)
	left_M1 = false;
	left_M2 = false;
	isFirstLeft = false; //Variable um festzustellen, mit welchem Bein der erste Schritt gemacht wurde.
	isSecondLeft = false;//Variable um festzustellen, mit welchem Bein der erste Schritt gemacht wurde.

	if (pos_vector_foot_left_new[2] < 0)
	{
		left_FORWARD_old = true;
		left_FORWARD_new = true;
		pos_vector_foot_left_old = pos_vector_foot_left_new[2];
	}

	else if (pos_vector_foot_left_new[2] >= 0)
	{
		left_FORWARD_old = false;
		left_FORWARD_new = false;
		pos_vector_foot_left_old = pos_vector_foot_left_new[2];
	}

	else
		cerr<<"the left footvectors are not avaible"<<endl;

}

void AGENT::SettingRightFootVariables()
{
	max_fw_pos_right= 0.0;
	max_bw_pos_right= 0.0;
	max_fw_angle_right = 0.0;
	max_bw_angle_right = 0.0;
	angle_difference_right = 0;
	rightStep = 0;//Zähler der Schritte (li / re)
	right_M1 = false;
	right_M2 = false;
	isFirstRight = false;//Variable um festzustellen, mit welchem Bein der erste Schritt gemacht wurde.
	isSecondRight = false;//Variable um festzustellen, mit welchem Bein der erste Schritt gemacht wurde.

	if (pos_vector_foot_right_new[2] < 0)
	{
		right_FORWARD_old = true;
		right_FORWARD_new = true;
		pos_vector_foot_right_old = pos_vector_foot_right_new[2];
	}
	
	else if (pos_vector_foot_right_new[2] >= 0)
	{
		right_FORWARD_old = false;
		right_FORWARD_new = false;
		pos_vector_foot_right_old = pos_vector_foot_right_new[2];
	}

	else
		cerr<<"the right footvectors are not avaible"<<endl;
}

/////////////////////////////////////////////////////////////////////////////////////
// Controll if the foot is moving forwards or backwards
void AGENT::FeetForwardsOrBackwards()
{
	if (pos_vector_foot_left_new[2] > pos_vector_foot_left_old)
	{
		left_FORWARD_new = false;
		pos_vector_foot_left_old = pos_vector_foot_left_new[2];
	}

	else if (pos_vector_foot_left_new[2] < pos_vector_foot_left_old)
	{
		left_FORWARD_new = true;
		pos_vector_foot_left_old = pos_vector_foot_left_new[2];
	}

	else if (pos_vector_foot_left_new[2] == pos_vector_foot_left_old)
	{
		left_FORWARD_new = left_FORWARD_old;
		pos_vector_foot_left_old = pos_vector_foot_left_new[2];
	}
	else
		cerr<<"there's a problem with Variable pos_vector_foot_left_new and ..._old";


	if (pos_vector_foot_right_new[2] > pos_vector_foot_right_old)
	{
		right_FORWARD_new = false;
		pos_vector_foot_right_old = pos_vector_foot_right_new[2];
	}

	else if (pos_vector_foot_right_new[2] < pos_vector_foot_right_old)
	{
		right_FORWARD_new = true;
		pos_vector_foot_right_old = pos_vector_foot_right_new[2];
	}
	
	else if (pos_vector_foot_right_new[2] == pos_vector_foot_right_old)
	{
		right_FORWARD_new = right_FORWARD_old;
		pos_vector_foot_right_old = pos_vector_foot_right_new[2];
	}
	else
		cerr<<"there's a problem with Variable pos_vector_foot_right_new and ..._old";
}

///////////////////////////////////////////////////////
//check if the foot is changing the movement direction 
void AGENT::FeetChangingDirection()
{
	if(!(left_FORWARD_new == left_FORWARD_old))
	{
		GetLeftFootMaximum();
	}

	if(!(right_FORWARD_new == right_FORWARD_old))
	{
		GetRightFootMaximum();
	}
}

///////////////////////////////////////////////////////////
//Get the maximal position in forward and backward position
void AGENT::GetLeftFootMaximum()
{
	if(pos_vector_foot_left_new[2] < MarkPoint1)
	{
		if(pos_vector_foot_left_new[2] < max_fw_pos_left)
		{
			directionBuffer = 0;
			MdtLimitID limitID = MdtHingeGetLimit(joints[6]);
			max_fw_angle_left = (double)MdtLimitGetPosition(limitID);
			max_fw_pos_left = pos_vector_foot_left_new[2];
//			cout<<"fw_left"<<max_fw_pos_left<<endl;
//			cin>>nextstep;
			left_M1 = true;
		}

	}

	else if(pos_vector_foot_left_new[2] > MarkPoint2)
	{
		if(pos_vector_foot_left_new[2] > max_bw_pos_left)
		{
			directionBuffer = 0;
			MdtLimitID limitID = MdtHingeGetLimit(joints[6]);
			max_bw_angle_left = (double)MdtLimitGetPosition(limitID);
			max_bw_pos_left = pos_vector_foot_left_new[2];
//			cout<<"bw_left"<<max_bw_pos_left<<endl;
//			cin>>nextstep;
			left_M2 = true;
			
		}
	}
}

///////////////////////////////////////////////////////////
//Get the maximal position in forward and backward position
void AGENT::GetRightFootMaximum()
{
	if(pos_vector_foot_right_new[2] < MarkPoint1)
	{
		if(pos_vector_foot_right_new[2] < max_fw_pos_right)
		{
			directionBuffer = 0;
			MdtLimitID limitID = MdtHingeGetLimit(joints[3]);
			max_fw_angle_right = (double)MdtLimitGetPosition(limitID);
			max_fw_pos_right = pos_vector_foot_right_new[2];
//			cout<<"fw_right"<<max_fw_pos_right<<" ; "<<right_FORWARD_new<<";"<<right_FORWARD_old<<endl;
//			cin>>nextstep;
			right_M1 = true;

		}

	}

	else if(pos_vector_foot_right_new[2] > MarkPoint2)
	{
		if(pos_vector_foot_right_new[2] > max_bw_pos_right)
		{
			directionBuffer = 0;
			MdtLimitID limitID = MdtHingeGetLimit(joints[3]);
			max_bw_angle_right = (double)MdtLimitGetPosition(limitID);
			max_bw_pos_right = pos_vector_foot_right_new[2];
//			cout<<"bw_right"<<max_bw_pos_right<<endl;
//			cin>>nextstep;
			right_M2 = true;

		}
	}
}

void AGENT::LeftFootStep()
{
	if((leftStep == 0)&&(rightStep == 0)) // find out which is the first leg doing a step, calculating this step and setting the flag
	{
		StepDistanceLeft();
		isFirstLeft = true; //flags, have to be set after the first step only!
		isSecondRight = true; 
//		cout<<"left is the first to take a step"<<" ; "<<"stepdistance: "<<left_step_distance<<endl;cout<<"right_FORWARD_new"<<right_FORWARD_new<<"left_FORWARD_new"<<left_FORWARD_new<<endl;
	}
	else if (isFirstLeft)  //if it was the first one, it only has to be calculated if in the meantime the other leg has also done a step -> the stepvariable is the same again
	{
					
		if(leftStep == rightStep)//&&(!(right_FORWARD_new==left_FORWARD_new)))
		{
			StepDistanceLeft();
//			cout<<"left took a step (first)"<<" ; "<<"stepdistance : "<<left_step_distance<<endl;cout<<"right_FORWARD_new"<<right_FORWARD_new<<"left_FORWARD_new"<<left_FORWARD_new<<endl;
		}
		else
		{
			SWINGING_LEG = true;
//			cout<<"erste Möglichkeit"<<endl;
		}				
	
	}
	else if(isSecondLeft)// if it was not the first leg, it has to do the next step
	{
		if(leftStep == rightStep-1)//&&(!(right_FORWARD_new==left_FORWARD_new)))
		{
			StepDistanceLeft();
//			cout<<"left took a step (second)"<<" ; "<<"stepdistance : "<<left_step_distance<<endl;cout<<"right_FORWARD_new"<<right_FORWARD_new<<"left_FORWARD_new"<<left_FORWARD_new<<endl;
		}
		else
		{
			SWINGING_LEG = true;
//			cout<<"zweite Möglichkeit"<<endl;
		}				
	}
}

void AGENT::RightFootStep()
{
	if((leftStep == 0)&&(rightStep == 0)) // find out which is the first leg doing a step, calculating this step and setting the flag
	{
		StepDistanceRight();
		isFirstRight = true; //flags, have to be set after the first set only!
		isSecondLeft = true;
//		cout<<"right was first to take a step"<<" ; "<<"stepdistance : "<<right_step_distance<<endl;cout<<"right_FORWARD_new"<<right_FORWARD_new<<"left_FORWARD_new"<<left_FORWARD_new<<endl;
	}

	else if (isFirstRight) // first step taken by the right foot
	{
		if(rightStep == leftStep)//&&(!(right_FORWARD_new==left_FORWARD_new)))
		{
			StepDistanceRight();
//			cout<<"right took a step (first)"<<" ; "<<"stepdistance : "<<right_step_distance<<endl;cout<<"right_FORWARD_new"<<right_FORWARD_new<<"left_FORWARD_new"<<left_FORWARD_new<<endl;
		}
		else
		{
			SWINGING_LEG = true;
//			cout<<"dritte Möglichkeit"<<endl;
		}
	}

	else if(isSecondRight)
	{
		if(rightStep == leftStep-1)//&&(!(right_FORWARD_new==left_FORWARD_new)))
		{
			StepDistanceRight();
//			cout<<"right took a step (second) "<<"step dist"<<right_step_distance<<endl;cout<<"right_FORWARD_new"<<right_FORWARD_new<<"left_FORWARD_new"<<left_FORWARD_new<<endl;
		}
		else
		{
			SWINGING_LEG = true;
//			cout<<"vierte Möglichkeit"<<endl;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////
//calculates the stepdistance as well as the total distance of the foot, resets the variable to starting situation
void AGENT::StepDistanceLeft()
{
	left_step_distance = - max_fw_pos_left + max_bw_pos_left;
	left_foot_distance += left_step_distance;
	max_fw_pos_left = 0.0;
	max_bw_pos_left = 0.0;
	left_M1 = false;
	left_M2 = false;
//	cout<<" l "<<leftStep<<endl;
	leftStep++;
//	cout<<" l "<<leftStep<<endl;
}
/////////////////////////////////////////////////////////////////////////////////////////////
//calculates the stepdistance as well as the total distance of the foot, resets the variable to starting situation
void AGENT::StepDistanceRight()
{
	right_step_distance = - max_fw_pos_right + max_bw_pos_right;	
	right_foot_distance += right_step_distance;
	max_fw_pos_right = 0.0;
	max_bw_pos_right = 0.0;
	right_M1 = false;
	right_M2 = false;
//	cout<<" r "<<rightStep<<endl;
	rightStep++;
//	cout<<" r "<<rightStep<<endl;
/*	if (rightStep>12)
		{cin>>nextstep;}*/
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//Reset the Step Distance Parameter of both feet
void AGENT::StepDistanceReset()
{
	right_step_distance = 0.0;
	left_step_distance = 0.0;
}

void AGENT::OutOfPhase()
{
	if(directionBuffer < DIRECTIONBUFFER)
	{
		BufferIncrease();
		directionChange = true;
	}

	else if(directionBuffer >= DIRECTIONBUFFER)
	{
		outOfPhase=true;
	}
	
}
///////////////////////////////////////////////////////////////////////////////////////////////
// increasing the buffer status
void AGENT::BufferIncrease()
{
	directionBuffer++;
}

void AGENT::DirectionControll()
{
	OutOfPhase();
}
////////////////////////////////////////////////////////////////////////////////////////////
//calculating the angles difference of the hip
void AGENT::AngleDifferenceLeft()
{
	angle_difference_left = fabs(max_bw_angle_left) + fabs(max_fw_angle_left);
}
	

void AGENT::AngleDifferenceRight()
{
	angle_difference_right = fabs(max_bw_angle_right) + fabs(max_fw_angle_right);
}
//////////////////////////////////////////////////////////////////////////////////////
//chechs if the angle difference is not higher then 20%
bool AGENT::AngleDiffCompare()
{
	AngleDifferenceLeft();
	AngleDifferenceRight();
	bool returnvalue;
	if ((angle_difference_left)==(angle_difference_right))
		returnvalue = true;
	else if (angle_difference_right > angle_difference_left)
	{
		if (angle_difference_right*0.8> angle_difference_left)
			returnvalue = false;
		else
			returnvalue = true;
	}
	else
	{
		if (angle_difference_left*0.8> angle_difference_right)
			returnvalue = false;
		else 
			returnvalue = true;
	}
	angle_difference_left = 0;
	angle_difference_right = 0;
	return returnvalue;
}
#endif