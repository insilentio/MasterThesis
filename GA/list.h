/* ---------------------------------------------------
   FILE:     list.h
	AUTHOR:   Josh Bongard
	DATE:     October 3, 2000
	FUNCTION: This class contains all information for
				 specifying a list data structure
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _LIST_H
#define _LIST_H

#include "element.h"

class LIST {

public:
	int listType;
	ELEMENT *root;
	ELEMENT *last;

private:
	int nextAvailableGenomeID;
	int length;

public:
	LIST(int newListType);
	~LIST(void);
	void    AddElement(void);
	void	  AddElement(int genomeID);
	void    AddElement(ELEMENT *newElement);
	void    Append(ELEMENT *newElement);
	void    GACopyGenomeAndAppend(ELEMENT *templateGenome);
	void    DeleteFirst(void);
	void    DeleteLast(void);
	void    Display(void);
	void    Display(ofstream outFile);
	ELEMENT *Get(int index);
	float   GetAvgFitness(void);
	float   GetAvgPassiveness(void);
	float   GetAvgVCOM(void);
	ELEMENT *GetFirst(void);
	int     IsEmpty(void);
	int     Length(void);
	ELEMENT *Next(ELEMENT *currElement);
	void    Sort(int currGen);
	ELEMENT *TourneyWinner(void);

private:
	ELEMENT *FindPrev(ELEMENT *el);
	void    GAInitialize(void);
	void    SwitchElements(ELEMENT *el, ELEMENT *nextEl);
};

#endif