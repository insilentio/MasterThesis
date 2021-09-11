/* ---------------------------------------------------
   FILE:     constants.h
	AUTHOR:   Josh Bongard (modified by Chandana Paul)
	DATE:     October 3, 2000
	FUNCTION: This class contains all of the constants
			    for this project.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _CONSTANTS_H
#define _CONSTANTS_H


// ---------------------------------------------------
//				MathEngine specific constants
// ---------------------------------------------------

float STEP_SIZE			= (float)0.008;

MeReal floorDims[3] = { 30.0f, 0.05f, 30.0f };
float GROUND_MASS		= 1000.0;
float MASS_ADDITION	= 1.0;

MeReal gravity[3] = { 0.0f, -9.80f, 0.0f };
//MeReal gravity[3] = { 0.0f, -5.0f, 0.0f };

float orange[3] = { 1.0f, 0.4f, 0.0f };
float green[3] = { 0.0f, 1.0f, 0.0f };
float red[3]  = {1.0f, 0.0f, 0.0f };
float purple[3] = {1.0f, 0.0f, 1.0f };
float blue[3] = { 0.0f, 0.598f, 0.797f };
float white[3] = {1.0f, 1.0f, 1.0f};

long int MEMORY_TO_USE	= 1000000;

float MOTOR_STRENGTH	= 10.0;
float MOTOR_DAMPING		= 1.0;
float MOTOR_RESTITUTION = 0.0;
float MOTOR_STIFFNESS	= 10.0;

float UPPER_HINGE_LIMIT	= PI / 2.0;

int   MAX_BODIES		= 30;
int	  MAX_BODIES_ALLOC	= MAX_BODIES+1;

MeReal groundTransform[4][4] =
    { {1, 0, 0, 0}, {0, 0, -1, 0}, {0, 1, 0, 0}, {0, 0, 0, 1} };

// ---------------------------------------------------
//				   Body specific constants
// ---------------------------------------------------

int   NUM_CONNECTORS	   = 6;
float DEFAULT_RADIUS	   = 0.2;
float DEF_CONNECTOR_MASS   = DEFAULT_RADIUS*20.0;

int	  NUM_LEG_SEGMENTS		= 7;
float DEF_LL_LENGTH			= DEFAULT_RADIUS * 8.0;
float DEF_UL_LENGTH			= DEF_LL_LENGTH;
float DEF_LL_MASS			= DEF_LL_LENGTH*10.0;
float DEF_UL_MASS			= DEF_UL_LENGTH*20.0;
float DEF_LL_WIDTH			= DEFAULT_RADIUS / 2.0;
float DEF_UL_WIDTH			= DEFAULT_RADIUS / 4.0 * 3.0;
float DEF_FOOT_LENGTH		= DEFAULT_RADIUS * 4.0; //4.0
float DEF_FOOT_MASS			= DEF_FOOT_LENGTH * 20.0;
float DEF_FOOT_WIDTH        = DEFAULT_RADIUS*2.0;
float DEF_FOOT_THICKNESS	= DEFAULT_RADIUS / 2.0;


int NUM_BODY_PARTS		   = NUM_CONNECTORS + NUM_LEG_SEGMENTS;

float BODY_WIDTH		   = DEF_LL_LENGTH;
float DEF_WAIST_MASS	   = BODY_WIDTH*70.0*50.0;      // 60% of the body's mass is above the waist
float DEF_WAIST_WIDTH	   = DEFAULT_RADIUS;

int NUM_JOINTS			   = 12; //10

float HIP_PITCH_MAXIMUM		= PI/9.0;
float HIP_PITCH_MINIMUM		= -HIP_PITCH_MAXIMUM;

float HIP_ROLL_MAXIMUM		= PI/5.0;
float HIP_ROLL_MINIMUM		= -HIP_ROLL_MAXIMUM;

float KNEE_MAXIMUM			= PI/2.1;
float KNEE_MINIMUM			= 0.01;

float ANKLE_MAXIMUM			= PI/3.0; // pi/3
float ANKLE_MINIMUM			= -PI/3.0; // -pi/3

float DAMPING               = 1000;
float STIFFNESS				= 2;
int	  FORCE_CONTROL         = 1; //1 for force control, 0 for posture control



// ---------------------------------------------------
//				Virtual world specific constants
// ---------------------------------------------------

float HEIGHT_OFFSET = 0.1; //0.0 --> agent can touch the ground //1.0 can not touch
float TARGET_SIZE= DEFAULT_RADIUS;
float TARGET_DIST = 0;//floorDims[2]/2.0;
int FRICTION = 0;
 
// ----------------------------------------------------------------
//			  MathEngine to NNetview interface specific constants
// ----------------------------------------------------------------

long INPUTCOUNT  = 12;
long  OUTPUTCOUNT = 38;


// ---------------------------------------------------
//			  Genetic algorithm specific constants
// ---------------------------------------------------
int FUNCTION_AS_READER	  = false;

int FIXED_MORPHOLOGY		  = false;
int USE_PASSIVE_DYNAMICS  = false;
int USE_BLOCKS				  = true;

int FUNCTION_AS_RESTARTER = false;

int POPULATION_SIZE       = 300;
int NUM_GENERATIONS		  = 4000;

int MORPH_PARAMS			  = 8;

int	GA_WEIGHTSCOUNT = 327;//still to define!!it is just a guess

int GENE_RANG             = GA_WEIGHTSCOUNT;
int FLOATING_PT_PRECISION = 2;

float WORST_FITNESS		  = 1000.0;

float CULL_FRACTION		  = 0.5;
float CROSS_FRACTION		  = 0.25;

int   NUM_TO_CULL			  = (int)(CULL_FRACTION*POPULATION_SIZE);
int   NUM_TO_KEEP			  = POPULATION_SIZE - NUM_TO_CULL;

float	AVG_NUM_OF_MUTS     = 20; // Avg. number of mutations per genome

float	MUTATION_INCREMENT   = 0.01;

int   TOURNEY_SIZE			= 2;

int    NUM_BINDING_SITES  = 50;
float  BINDING_SITE_PROB  = (float)NUM_BINDING_SITES / (float)GA_WEIGHTSCOUNT;//DEF_GENOME_LENGTH;

int    GENE_LENGTH		  = 8;

int    UNEVEN_CROSSOVER	  = false;

int LENGTH = GA_WEIGHTSCOUNT;

int MAXRUNS                = 2000;

bool PASSIVESTAGE          = true;
//---------------------------------------------------
//Marken welche für die Schrittweitenberechnung benötigt werden

float	MarkPoint1			= -2*DEFAULT_RADIUS;
float	MarkPoint2			= (2*DEFAULT_RADIUS);

//----------------------
//don't know what they are for
int EL_GENOME =0; 
int GENOME_LIST =0;

 //-----------------------------------------------------------------
// those numbers below can be read out of the exportfile, best way is counting it in a Excel-File


 const int JOINT_ANGLES_COUNT		=			12;
 const int MOTORCOUNT				=			12;
 int DATACOUNT				=			JOINT_ANGLES_COUNT + MOTORCOUNT;
 int EXPFILECOUNT			=			100;
 int AREACOUNT				=			61;
 int CONNECTIONCOUNT		=			602;
 int UNITCOUNT				=			342;
 int TICK_START				=			1;
 int TICK_BIS_REFLEX_START	=			100;
 int MN_LEFT_KNEE_FLEX		=			56; //this number is needed in NtfMotorWrite
 int MN_LEFT_KNEE_EXT		=			MN_LEFT_KNEE_FLEX+1;		//
 int MN_LEFT_HIP_FLEX		=			MN_LEFT_KNEE_FLEX+2;	  //
 int MN_LEFT_HIP_EXT		=			MN_LEFT_KNEE_FLEX+3;	//
 int MN_RIGHT_HIP_FLEX		=			MN_LEFT_KNEE_FLEX+4;  //
 int MN_RIGHT_HIP_EXT		=			MN_LEFT_KNEE_FLEX+5;//those numbers are not needed in NtfMotorWrite, but probably useful if you like to print out Motorneuron Activity
 int MN_RIGHT_KNEE_FLEX		=			MN_LEFT_KNEE_FLEX+6;	//
 int MN_RIGHT_KNEE_EXT		=			MN_LEFT_KNEE_FLEX+7;		//
 int MN_LEFT_ANKLE_FLEX		=			MN_LEFT_KNEE_FLEX+8;			//
 int MN_LEFT_ANKLE_EXT		=			MN_LEFT_KNEE_FLEX+9;				//
 int MN_RIGHT_ANKLE_FLEX	=			MN_LEFT_KNEE_FLEX+10;					//
 int MN_RIGHT_ANKLE_EXT		=			MN_LEFT_KNEE_FLEX+11;//this is the endpoint in NtfMotorWrite, also needed
 int SENSOR_START			=			290;//needed in NtfSensorRead	
 int SENSOR_END				=			327;//needed in NtfSensorRead

 int ANGLES_MN_WRITE		=			false;
 int SENSOR_ACTIVATE		=			true;
 int REFLEX_ACTIVATE		=			false; 
 int GA_FITNESS_ACTIVATE    =           true;
 int WEIGTHS_INFOS			=			false; // if true, the created weight array and the suUID and suAID are saved in a txt file

 int DIRECTIONBUFFER		=			35;	//within that many ticks the opposing leg should also change the moving direction

 int REFLEX_TIME			=			5;
 float REFLEX_STIMULUS		=			1;

 int CHECK_ORIGINAL_WEIGHTS =			false; //true to write the original weights that should be changed to a dat file.
 int WRITE_ORIGINAL_CONNECTION =		false; //write the whole Connection.* struct to a txt file

 int EXP					=			4; // The exponent of the characteristic line of the spindel

#endif
	