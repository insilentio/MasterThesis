/* ---------------------------------------------------
   FILE:     growGA.h
	AUTHOR:   Josh Bongard
	DATE:     October 20, 2000
	FUNCTION: This class contains all information for
				 a population of variable-length genotypes

 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _GROW_GA_H
#define _GROW_GA_H

#include "list.h"
#include "fstream.h"

class GROW_GA {

public:
	int popSize;
	int currGeneration;
	int finished;
	ELEMENT *currGenome;

private:
	LIST *genomes;
	ofstream fitnessFile;
	ofstream popFile;
	ofstream bestAgentFile;
	float bestFitness;
	float avgFitness;
	float bestPassiveness;
	float avgPassiveness;
	float bestVCOM;
	float avgVCOM;

public:
	GROW_GA(void);
	~GROW_GA(void);
	void   Display(void);
	void   DisplayCurrentGenome(void);
	GENOME *GetCurrentGenome(void);
	void   NextGenome(void);
	void   SetFitness(float fitValue);
	void   SetPassiveness(float passFract);
	void   SetVCOM(float verticalCentreOfMass);

private:
	void   Cross(void);
	void   Cull(void);
	void   Fill(void);
	float  GetAvgFitness(void);
	float  GetAvgPassiveness(void);
	float  GetAvgVCOM(void);
	void   NextGeneration(void);
	void   WritePopulationReport(void);
	void   WriteToFile(void);
};

#endif