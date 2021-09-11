/* ---------------------------------------------------
   FILE:     list.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 3, 2000
	FUNCTION: This class contains all information for
				 defining a list data structure.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _LIST_CPP
#define _LIST_CPP

#include "list.h"
#include "simParams.h"
#include "mathWorld.h"
#include "growGA.h"

extern int GENOME_LIST;

extern int EL_GENOME;

extern int NUM_TO_KEEP;
extern int TOURNEY_SIZE;

extern SIM_PARAMS *simParams;
extern MATH_WORLD *mathWorld;
extern GROW_GA		*growGA; 

extern int        POSSIBLE_DIRECTIONS;
extern int        POPULATION_SIZE;
extern int        FUNCTION_AS_READER;
extern float      WORST_FITNESS;
extern int        NUM_GENERATIONS;

LIST::LIST(int newListType) {

//	printf("Creating a list.\n");

	listType = newListType;

	root = NULL;
	last = NULL;
	length = 0;

	nextAvailableGenomeID = 0;

	if ( listType == GENOME_LIST )
		GAInitialize();
}

LIST::~LIST(void) {

	while ( root != NULL )
		DeleteFirst();
}

void LIST::AddElement(void) {

	ELEMENT *newElement;

	if ( listType == GENOME_LIST ) {
		newElement = new ELEMENT(EL_GENOME,nextAvailableGenomeID);
		nextAvailableGenomeID++;
	}

	AddElement(newElement);
}

void LIST::AddElement(int genomeID) {

	ELEMENT *newElement = 
		new ELEMENT(EL_GENOME,genomeID);

	AddElement(newElement);
}

void LIST::AddElement(ELEMENT *newElement) {

	if ( root == NULL ) {
		root = newElement;
		last = newElement;
	}
	else {
		last->next = newElement;
		last = newElement;
	}
	length++;
}

void LIST::Append(ELEMENT *newElement) {

	AddElement(newElement);
}

void LIST::DeleteFirst(void) {

	if ( root != NULL ) {

		if ( root == last ) {
			delete root;
			root = NULL;
			last = NULL;
		}
		else {
			ELEMENT *tempPtr = root;
			root = root->next;
			delete tempPtr;
		}
		length--;
	}
}

void LIST::DeleteLast(void) {

	if ( !IsEmpty() ) {
		if ( root == last ) {
			delete root;
			root = NULL;
			last = NULL;
		}
		else {
			ELEMENT *tempPtr = root;
			while ( tempPtr->next != last )
				tempPtr = tempPtr->next;
			delete last;
			last = tempPtr;
		}
		length--;
	}
}

void LIST::Display(void) {

	if ( root != NULL ) {
		ELEMENT *tempPtr = root;

		while ( tempPtr != NULL ) {

			tempPtr->Display();
			tempPtr = tempPtr->next;
		}
	}
}

void LIST::Display(ofstream outFile) {

	if ( root != NULL ) {
		ELEMENT *tempPtr = root;

		while ( tempPtr != NULL ) {

			tempPtr->Display(outFile);
			tempPtr = tempPtr->next;
		}
	}
}

void LIST::GACopyGenomeAndAppend(ELEMENT *templateGenome) {

	ELEMENT *newGenome = new ELEMENT(templateGenome,nextAvailableGenomeID++);
	Append(newGenome);
}

ELEMENT *LIST::Get(int index) {

	ELEMENT *tempPtr = root;
	ELEMENT *foundElement = NULL;

	int currElement = 0;
	int found = false;

	while ( (tempPtr != NULL) &&
			  (!found) ) {

		if ( currElement == index ) {
			found = true;
			foundElement = tempPtr;
		}
		currElement++;
		tempPtr = tempPtr->next;
	}

	return( foundElement );
}

float LIST::GetAvgFitness(void) {

	float totalFitness = 0.0;

	ELEMENT *tempPtr = root;

	while ( tempPtr != NULL ) {

		totalFitness = totalFitness + tempPtr->GetFitness();

		tempPtr = tempPtr->next;
	}

	return( totalFitness / length );
}

float LIST::GetAvgPassiveness(void) {

	float totalPassiveness = 0.0;

	ELEMENT *tempPtr = root;

	while ( tempPtr != NULL ) {

		totalPassiveness = totalPassiveness + tempPtr->GetPassiveness();

		tempPtr = tempPtr->next;
	}

	return( totalPassiveness / length );
}

float LIST::GetAvgVCOM(void) {

	float totalVCOM = 0.0;

	ELEMENT *tempPtr = root;

	while ( tempPtr != NULL ) {

		totalVCOM = totalVCOM + tempPtr->GetVCOM();

		tempPtr = tempPtr->next;
	}

	return( totalVCOM / length );
}

ELEMENT *LIST::GetFirst(void) {

	return( root );
}

int  LIST::IsEmpty(void) {

	return ( length == 0 );
}

int  LIST::Length(void) {

	return( length );
}

ELEMENT *LIST::Next(ELEMENT *currElement) {

	if ( currElement != NULL )
		return( currElement->next );
	else
		return( NULL );
}

void LIST::Sort(int currGen) {

   int bound = length-1;
   int t;
   int j;
   int tmp;
	float firstFitness, secondFitness;
	float firstPassive, secondPassive;

	ELEMENT *firstEl, *secondEl, *tempEl;

   do {
       t = 0;
       for(j = 0; j < bound; ++j) {

			  firstEl = Get(j);
			  secondEl = Get(j+1);
			  firstFitness = firstEl->genome->GetFitness();
			  secondFitness = secondEl->genome->GetFitness();
			  firstPassive = firstEl->genome->GetPassivity();
			  secondPassive = secondEl->genome->GetPassivity();

           if( firstFitness <= secondFitness ) {
					SwitchElements(firstEl,secondEl);
					t = j;
           }
       }
       bound = t;
   } while (t != 0);
}

ELEMENT *LIST::TourneyWinner(void) {

	ELEMENT *currentWinner;
	int winnerIndex, contenderIndex;
	int numCompetitions = 0;

	winnerIndex = simParams->Rand(0,NUM_TO_KEEP-1);
	currentWinner = Get(winnerIndex);

	while ( numCompetitions < TOURNEY_SIZE ) {

		contenderIndex = simParams->Rand(0,NUM_TO_KEEP-1);

		if ( contenderIndex < winnerIndex ) {
			winnerIndex = contenderIndex;
			currentWinner = Get(winnerIndex);
		}

		numCompetitions++;
	}

	return( currentWinner );
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

ELEMENT *LIST::FindPrev(ELEMENT *el) {

	if ( root != NULL ) {

		ELEMENT *tempPtr = root;

		while ( (tempPtr != NULL) && (tempPtr->next != el) ) {
		
			tempPtr = tempPtr->next;
		}
		return( tempPtr );
	}
	else
		return( NULL );
}

void LIST::GAInitialize(void) {

	int g;

	if ( FUNCTION_AS_READER )
		AddElement(0);
	else
		for( g=0; g<POPULATION_SIZE; g++ )
			AddElement();
}

void LIST::SwitchElements(ELEMENT *el, ELEMENT *nextEl) {

	if ( root == el ) {
		el->next = nextEl->next;
		root = nextEl;
		nextEl->next = el;
	}
	else {
		ELEMENT *prevEl = FindPrev(el);
		el->next = nextEl->next;
		nextEl->next = el;
		prevEl->next = nextEl;
	}

	if ( last->next != NULL )
		last = last->next;
}

#endif