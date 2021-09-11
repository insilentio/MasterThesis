/* ---------------------------------------------------
   FILE:     genome.h
	AUTHOR:   Josh Bongard
	DATE:     October 20, 2000
	FUNCTION: This class contains all information for
				 a single genome of a genetic algorithm.
 -------------------------------------------------- */

#include "stdafx.h"
#include "fstream.h"

#ifndef _GENOME_H
#define _GENOME_H

class GENOME {

public:
	int    ID;
	int    length;
	int    evaluated;
	float  *genes;
	float  *weights;
	float  fitness;
	float  passiveness;
	float  verticalCentreOfMass;

private:
	ifstream inFile;
	ofstream outFile;

public:
	GENOME(int newID);
	GENOME(GENOME *templateGenome, int newID, int copyingIntoNewNode);
	~GENOME(void);
	void   Cross(GENOME *secondParent);
	void   Display(int showGenome);
	void   Display(ofstream outFile);
	float  Gene(int index);
	float  GetFitness(void);
	float  GetPassivity(void);
	void   MutateGene(int geneIndex);
	void   ReadFromFile(void);
	void   SetFitness(float fitValue);
	void   WriteToFile(void);

private:
	void   CrossUnevenly(GENOME *secondParent);
};

#endif