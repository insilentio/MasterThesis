/* ---------------------------------------------------
   FILE:     element.h
	AUTHOR:   Josh Bongard
	DATE:     October 2, 2000
	FUNCTION: This class contains all information for
				 the specification of a single element
				 in a list.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _ELEMENT_H
#define _ELEMENT_H

#include "genome.h"

class ELEMENT {

public:
	GENOME  *genome;
	ELEMENT *next;

private:
	int ID;

public:
	ELEMENT(int newElType, int newID);
	ELEMENT(ELEMENT *otherGenome, int newID);
	~ELEMENT(void);
	void  Cross(ELEMENT *secondParent);
	void  Display(void);
	void  Display(int showGenome);
	void  Display(ofstream outFile);
	float GetFitness(void);
	int   GetLength(void);
	float GetPassiveness(void);
	float GetVCOM(void);
	void  WriteToFile(void);
};

#endif