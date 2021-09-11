/* ---------------------------------------------------
   FILE:     growGA.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 20, 2000
	FUNCTION: This class contains all information for
				 a population of variable-length genotypes
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _GROW_GA_CPP
#define _GROW_GA_CPP

#include "growGA.h"
#include "simParams.h"

extern SIM_PARAMS *simParams;

extern int   NUM_GENERATIONS;
extern float WORST_FITNESS;
extern int   NUM_EVALS;
extern int   EL_GENOME;
extern int   TOURNEY_SIZE;
extern int   FUNCTION_AS_READER;
//extern int   DEF_GENOME_LENGTH;
extern int	 GA_WEIGHTSCOUNT;
extern int   GENE_RANGE;

GROW_GA::GROW_GA(void) {

	char fileName[100];

	printf("Creating a GA population.\n");

	genomes = new LIST(EL_GENOME);

	popSize = genomes->Length();
	currGeneration = 0;
	finished = false;

	if ( FUNCTION_AS_READER )
		sprintf(fileName,"data/fit.dat");
	else
		sprintf(fileName,"data%d/fit.dat",simParams->runNumber);

	fitnessFile.open(fileName);
	fitnessFile.close();

	sprintf(fileName,"data%d/bestAgents.dat",simParams->runNumber);
	bestAgentFile.open(fileName);
	bestAgentFile.close();

	currGenome = genomes->GetFirst();
}

GROW_GA::~GROW_GA(void) {

	printf("Destroying a GA population.\n");

	delete genomes;
	currGenome = NULL;
}

void GROW_GA::Display(void) {

	printf("----------------------------\n");
	printf("Generation %d\n",currGeneration);
	genomes->Display();
	printf("----------------------------\n");
}

void GROW_GA::DisplayCurrentGenome(void) {

	printf("Gen %d of %d: ",currGeneration,NUM_GENERATIONS);

	if ( currGenome != NULL )
		currGenome->Display();
}

GENOME* GROW_GA::GetCurrentGenome(void) {

	if ( currGenome != NULL )
		return( currGenome->genome );
	else
		return( NULL );
}

void GROW_GA::NextGenome(void) {

	currGenome->genome->evaluated = true;

	currGenome = genomes->Next(currGenome);

	if ( currGenome == NULL )
			NextGeneration();
}

void GROW_GA::SetFitness(float fitValue) {

	currGenome->genome->fitness = fitValue;
}

void GROW_GA::SetPassiveness(float passFract) {

	currGenome->genome->passiveness = passFract;
}

void GROW_GA::SetVCOM(float verticalCentreOfMass) {

	currGenome->genome->verticalCentreOfMass = verticalCentreOfMass;
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void GROW_GA::Cross(void) {

	extern float CROSS_FRACTION;

	int numToCross = (int) (popSize * CROSS_FRACTION);

	if ( (numToCross%2) != 0 )
		numToCross--;

	ELEMENT *parentOne, *parentTwo;

	parentOne = genomes->Get(popSize - numToCross);

	while ( parentOne != NULL ) {

		parentTwo = genomes->Next(parentOne);

		parentOne->Cross(parentTwo);

		parentOne = genomes->Next(parentTwo);
	}
}

void GROW_GA::Cull(void) {

	extern int NUM_TO_CULL;

	int numKilled = 0;

	while ( numKilled < NUM_TO_CULL ) {
		genomes->DeleteLast();
		numKilled++;
		popSize--;
	}
}

void GROW_GA::Fill(void) {

	extern int NUM_TO_CULL;

	int genomesReplaced = 0;

	while ( genomesReplaced < NUM_TO_CULL ) {
		genomes->GACopyGenomeAndAppend( genomes->TourneyWinner() );
		genomesReplaced++;
		popSize++;
	}

	Cross();
}

float GROW_GA::GetAvgFitness(void) {

	return( genomes->GetAvgFitness() );	
}

float GROW_GA::GetAvgPassiveness(void) {

	return( genomes->GetAvgPassiveness() );	
}

float GROW_GA::GetAvgVCOM(void) {

	return( genomes->GetAvgVCOM() );	
}

void GROW_GA::NextGeneration(void) {

	genomes->Sort(currGeneration);

	WriteToFile();
	genomes->GetFirst()->Display();

	Cull();
	Fill();

	currGenome = genomes->GetFirst();
	currGenome->WriteToFile();

	while ( (currGenome != NULL) &&
			  (currGenome->genome->evaluated) )
		currGenome = genomes->Next(currGenome);

	if ( currGenome == NULL )
		finished = true;

	currGeneration++;

	if ( currGeneration == NUM_GENERATIONS )
		finished = true;
}

void GROW_GA::WritePopulationReport(void) {

	char popFileName[50];

	sprintf(popFileName,"data%d/gen%d.dat",simParams->runNumber,currGeneration);

	popFile.open(popFileName);

	popFile << "-------------------------------\n";
	popFile << "Generation: " << currGeneration << "\n\n";
	popFile << "Average fitness: " << avgFitness << "\n";
	popFile << "Best fitness: " << bestFitness << "\n\n";
	popFile << "Average passiveness: " << avgPassiveness << "\n";
	popFile << "Best passiveness: " << bestPassiveness << "\n\n";
	popFile << "Average vertical centre of mass: " << avgVCOM << "\n";
	popFile << "Best vertical centre of mass: " << bestVCOM << "\n\n";
	popFile << "-------------------------------\n";

	genomes->Display(popFile);

	popFile.close();
}

void GROW_GA::WriteToFile(void) {

	char fitFileName[50];

	if ( FUNCTION_AS_READER )
		sprintf(fitFileName,"data/fit.dat");
	else
		sprintf(fitFileName,"data%d/fit.dat",simParams->runNumber);

	fitnessFile.open(fitFileName,ios::app);

	avgFitness = GetAvgFitness();
	bestFitness = genomes->GetFirst()->GetFitness();

	avgPassiveness = GetAvgPassiveness();
	bestPassiveness = genomes->GetFirst()->GetPassiveness();

	avgVCOM = GetAvgVCOM();
	bestVCOM = genomes->GetFirst()->GetVCOM();

	fitnessFile << avgFitness << ";";
	fitnessFile << bestFitness << ";";
	fitnessFile << avgPassiveness << ";";
	fitnessFile << bestPassiveness << ";";
	fitnessFile << avgVCOM << ";";
	fitnessFile << bestVCOM << "\n";

	fitnessFile.close();

	bestAgentFile.open("data/bestAgents.dat",ios::app);
	bestAgentFile << genomes->GetFirst()->genome->ID << " ";
	bestAgentFile.close();

	WritePopulationReport();
}

#endif