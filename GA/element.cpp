/* ---------------------------------------------------
   FILE:     element.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 3, 2000
	FUNCTION: This class contains all information for
				 the definition of a single element in a
				 list.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _ELEMENT_CPP
#define _ELEMENT_CPP

#include "element.h"

extern int EL_NODE;
extern int EL_EDGE;
extern int EL_GENOME;

ELEMENT::ELEMENT(int newElType, int newID) {

	ID = newID;

	genome = new GENOME(newID);

	next = NULL;
}

ELEMENT::ELEMENT(ELEMENT *otherGenome, int newID) {

	ID = newID;

	genome = new GENOME(otherGenome->genome,newID,0);

	next = NULL;
}

ELEMENT::~ELEMENT(void) {

	if ( genome != NULL )
		delete genome;
}

void ELEMENT::Cross(ELEMENT *secondParent) {

	genome->Cross(secondParent->genome);
}

void ELEMENT::Display(void) {

	genome->Display(false);
}

void ELEMENT::Display(int showGenome) {

	genome->Display(showGenome);
}

void ELEMENT::Display(ofstream outFile) {

	genome->Display(outFile);
}

float ELEMENT::GetFitness(void) {

	return( genome->fitness );
}

int   ELEMENT::GetLength(void) {

	return( genome->length );
}

float ELEMENT::GetPassiveness(void) {

	return( genome->passiveness );
}

float ELEMENT::GetVCOM(void) {

	return( genome->verticalCentreOfMass );
}

void ELEMENT::WriteToFile(void) {

	if ( genome != NULL )
		genome->WriteToFile();
}

#endif