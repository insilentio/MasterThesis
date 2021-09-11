/* ---------------------------------------------------
   FILE:     channel.cpp
   AUTHOR:   Chandana Paul
   DATE:     January 11, 2001
   FUNCTION: This file defines the interface channels between MathEngine
             and Nnetview
 -------------------------------------------------- */

#include "stdafx.h"
#include "channel.h"
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>

extern long INPUTCOUNT;
extern long OUTPUTCOUNT;

/////////////////////////////////////////////////////////////////////////////
// Channel

/////////////////////////
//Data Transfer Functions

bool Channel::Dtf(int nMode)
{
	if ((bDtfInp)&&(bDtfOut))
	{
		//both not possible
		return false;
	}

	if (nMode == DTF_END)
	{
		inp.close();
		out.close();
		return true;
	}

	if ((bDtf)&&(bDtfInp)&&(!bDtfOut))
	{
		//input
		if (!inp.is_open())
		{ 
			//check input file existence
			inp.open(strDtfFileName, ios::in | ios::nocreate, filebuf::sh_write);
		}
		if (!inp.is_open()) 
		{
			//create input file
			inp.open(strDtfFileName, ios::in, filebuf::sh_write);
			inp.close();
			return false;
		}
	}

	if ((bDtf)&&(bDtfOut)&&(!bDtfInp)) 
	{
		//output
		if (!out.is_open()) 
		{
			out.open(strDtfFileName, ios::out, filebuf::sh_read);
		}
		if (!out.is_open()) 
		{
			return false;
		}
	} 
	
	return true;
}

bool Channel::DtfRead(float *nnvvalues)
{
	Dtf(DTF_SYNC);

	if ((bDtf)&&(bDtfInp))
	{
		if (inp.is_open())
		{
			char pCh[32];
			inp.seekp(0,ios::beg);
			//check unit counts
			inp.getline(pCh,sizeof(pCh));
			long l = atol(pCh);
			if (l != INPUTCOUNT)
			{
				//NNV
				inp.close();
				printf("error!\n");
				return false;
			}
			
			//read unit values
			int i = 0;
			while (i < l) 
			{
				if (inp.eof())
				{
					return false;
				}

				inp.getline(pCh,sizeof(pCh));
				nnvvalues[i] = atof(pCh);	
				i++;
			}
			
			inp.seekp(0,ios::beg);
		}
		
		else
		{
			inp.open(strDtfFileName, ios::in | ios::nocreate, filebuf::sh_write);
			return false;
		}
	}
	
	else
	{
		return false;
	}
	
	return true;
}

bool Channel::DtfWrite(long outputcount, float *outputvalues)
{
	Dtf(DTF_SYNC);
	
	if ((bDtf)&&(bDtfOut))
	{
		if (out.is_open())
		{
			out.seekp(0,ios::beg);
			out << outputcount << "\n";
			int i = 0;		
			while (i < outputcount)
			{
				out << outputvalues[i] << "\n";	
				i++;			
			}

			out.seekp(0,ios::beg);
		} 
		
		else
		{
			return false;
		}
	} 
	
	else 
	{
		return false;
	}
	
	return true;
}



/////////////////////////////////////////////////////////////////////////////
// Constructor and Destructor

// Constructor for areas, areas are arranged in a ring-list, so that every unit and every
// connection can access the whole structure, especially for deallocation-purposes

Channel::Channel(char filename[16], bool input)
{ 
	bDtf			= true;
	if (input == 1)
	{
	//	printf("Creating an Input Channel called %s\n", filename);
		bDtfInp	= true;
		bDtfOut = false;
	}

	else 
	{
	//	printf("Creating an Output Channel called %s\n", filename); 
		bDtfInp	= false;
		bDtfOut = true;
	}

	strcpy(strDtfFileName, filename);
	nAcr			= 100; //[ms]	has been 25 originally
}


// Destructor for areas
// integrity of linkage is implemented, but handling of the anchor has
// to be done by the user
Channel::~Channel()
{
  
	inp.close();
	out.close();
}