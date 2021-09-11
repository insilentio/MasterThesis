/* ---------------------------------------------------
   FILE:     genome.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 20, 2000
	FUNCTION: This class contains all information for
				 a single genome in a genetic algorithm.
 -------------------------------------------------- */

#include "stdafx.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"

#ifndef _GENOME_CPP
#define _GENOME_CPP

#include "genome.h"
#include "simParams.h"

extern SIM_PARAMS *simParams;
extern float		MUTATION_INCREMENT;
extern int			POSSIBLE_DIRECTIONS;
extern int          GENE_LENGTH;
extern int          TOTAL_CHEMS;
extern float		AVG_NUM_OF_MUTS;
//extern int        DEF_GENOME_LENGTH;	
extern int			GA_WEIGHTSCOUNT;	//defines the length of the genome
extern int          FLOATING_PT_PRECISION;
extern float        WORST_FITNESS;
extern int			UNEVEN_CROSSOVER;
extern int			FUNCTION_AS_READER;

GENOME::GENOME(int newID) 
{

	ID = newID;
	evaluated = 0;
	fitness = 0.0;
	passiveness = 0.0;
	bool isNegative;

	if ( FUNCTION_AS_READER )
		ReadFromFile();

	else 
	{
		length = GA_WEIGHTSCOUNT;	
		genes = new float[length];
		weights = new float[length];

		int g, a;
		int prec = (int)pow(10.0,FLOATING_PT_PRECISION);

		for ( g=0; g<length; g++ ) 
		{
			weights[g]= (float)simParams->GetWeights(g);
//			cout<<weights[g]<<" , "<<g<<endl;
			a=(int)simParams->CheckWeightRange(g);

			if (weights[g]<0)
				isNegative = true;
			else
				isNegative = false;
			
			if (isNegative == true)
			{
			
					genes[g] = (float)simParams->RandInt(a*prec,0);
					genes[g] = genes[g] / prec;
			
			}
			else
			{
				
					genes[g] = (float)simParams->RandInt(0,a*prec);
					genes[g] = genes[g] / prec;	
			

			}
//			cout<<genes[g]<<" , "<<g<<endl;
		}
//		cout<<genes[323]<<" , "<<"nach dem Beschreiben"<<endl;
	}
}

GENOME::GENOME(GENOME *templateGenome, int newID, int copyingIntoNewNode) {

	ID = newID;

	genes = new float[templateGenome->length];
	length = templateGenome->length;
	evaluated = 0;
	fitness = 0.0;
	passiveness = 0.0;

	int g;

	float mutProbability = AVG_NUM_OF_MUTS / length;

	for ( g=0; g<length; g++ ) {
		genes[g] = templateGenome->Gene(g);
		

		if ( (!copyingIntoNewNode) &&
			  (simParams->Rand(0.0,1.0) < mutProbability) )
			MutateGene(g);
	}
}

GENOME::~GENOME(void) {

	delete[] genes;
	genes = NULL;
}

void GENOME::Cross(GENOME *secondParent) {

	if ( !UNEVEN_CROSSOVER ) {

		int crossPoint = simParams->RandInt(1,length-1);
		int g;
		float tmp;

		for (g=crossPoint;g<length;g++) {
			tmp = secondParent->genes[g];
			secondParent->genes[g] = genes[g];
			genes[g] = tmp;
		}
	}
	else
		CrossUnevenly(secondParent);
}

void GENOME::Display(int showGenome) {

	int g;

	printf("Gen %d: [e: %d |f: %2.8f |p: %2.2f |l: %d]",
		ID,evaluated,fitness,passiveness,length);
	
	if ( showGenome ) {
		
		printf("[");

		for ( g=0; g<length; g++ )
			if ( g == (length-1) )
				printf("%1.2f",genes[g]);
			else
				printf("%1.2f ",genes[g]);

		printf("]\n");
	}
	else
		printf("\n");
}

void GENOME::Display(ofstream outFile) {

	outFile << "Genome " << ID << ": ";
	outFile << "[e: " << evaluated << " ";
	outFile << "f: " << fitness << " ";
	outFile << "p: " << passiveness << " ";
	outFile << "l: " << length << "\n";
}

float GENOME::Gene(int index) {

	if ( index < length )
		return( genes[index] );
	else
		return( 0.0 );
}

float GENOME::GetFitness(void) {

	return( fitness );
}

float GENOME::GetPassivity(void) {

	return( passiveness );
}

void GENOME::MutateGene(int geneIndex) 
{
	bool isNegative;

	int prec = (int)pow(10.0,FLOATING_PT_PRECISION);

	if (genes[geneIndex]<0)
		isNegative=true;

	else
		isNegative=false;

	
	if(isNegative==true)
	{ 
		while(genes[geneIndex]>=0)
		{
			genes[geneIndex] = (float)simParams->RandInt(0,prec);
			genes[geneIndex] = genes[geneIndex] / prec;
		}
	}
	else
	{	
		while(genes[geneIndex]<=0)
		{
			genes[geneIndex] = (float)simParams->RandInt(0,prec);
			genes[geneIndex] = genes[geneIndex] / prec;
		}
	}
	
}

void GENOME::ReadFromFile(void) {

	char fileName[50];

	sprintf(fileName,"data/ga%d.dat",ID);
	
	inFile.open(fileName);

	inFile >> length;
	genes = new float[length];
	int g;
	for ( g=0; g<length; g++ )
		inFile >> genes[g];

	inFile.close();
}

void  GENOME::SetFitness(float fitValue) {

	fitness = fitValue;
}

void GENOME::WriteToFile(void) {

	char outFileName[50];
	char charID[10];
	int g;

	sprintf(outFileName,"data%d/ga%d.dat",simParams->runNumber,ID);

	outFile.open(outFileName);

	outFile << length << " ";

	for ( g=0; g<length; g++ )
		outFile << genes[g] << " ";

	outFile << "\n" << fitness; //rf

	outFile.close();
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void GENOME::CrossUnevenly(GENOME *secondParent) {

	// This method is used to perform uneven crossover on
	// two genomes. This event usually leads to the two new
	// genomes have differing lengths both from each other,
	// and from the original two genomes. Thus the genetic
	// algorithm can exploit this mechanism to either lengthen
	// or shorten the genomes in the population.

	// Also, uneven crossover allows the genetic algorithm to
	// indirectly perform gene duplication: the same gene, if
	// present on both parents, can be copied twice into the new
	// genome, depending on the choice of the two crossover points.

	int crossPointOne, crossPointTwo;
	int newLengthOne, newLengthTwo;
	int g, newGeneIndex;
	float *newGenomeOne = NULL;
	float *newGenomeTwo = NULL;

	// Choose the crossover points, dependent on the lengths of
	// the genomes.
	crossPointOne = simParams->RandInt(1,length-1);
	crossPointTwo = simParams->RandInt(1,secondParent->length-1);
	
	// Calculate the lengths of the new genomes.
	newLengthOne = crossPointOne + (secondParent->length - crossPointTwo);
	newLengthTwo = crossPointTwo + (length - crossPointOne);

	newGenomeOne = new float[newLengthOne];
	newGenomeTwo = new float[newLengthTwo];

	newGeneIndex = 0;
	for ( g=0; g<=crossPointOne; g++ )
		newGenomeOne[newGeneIndex++] = genes[g];

	for ( g=(crossPointTwo+1); g<secondParent->length; g++ )
		newGenomeOne[newGeneIndex++] = secondParent->genes[g];

	newGeneIndex = 0;
	for ( g=0; g<=crossPointTwo; g++ )
		newGenomeTwo[newGeneIndex++] = secondParent->genes[g];

	for ( g=(crossPointOne+1); g<length; g++ )
		newGenomeTwo[newGeneIndex++] = genes[g];

	delete[] genes;
	genes = newGenomeOne;

	delete[] secondParent->genes;
	secondParent->genes = newGenomeTwo;

	length = newLengthOne;
	secondParent->length = newLengthTwo;
}

#endif