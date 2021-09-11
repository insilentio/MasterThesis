/* ---------------------------------------------------
    FILE:     channel.h
	AUTHOR:   Chandana Paul
	DATE:     January 11, 200§
	FUNCTION: This file is the header file, for defining 
	          the communication channels between MathEngine
			  and Nnetview
 -------------------------------------------------- */

#include <fstream.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////////////
// Channel document

enum {			//dtf
	DTF_SYNC,
	DTF_END
};

class Channel 
{
public:
	Channel(char filename[16], bool input);           // protected constructor used by dynamic creation
    ~Channel();

// Implementation
public:
	//////////////////////////////////////
	//Fundamental attributes and functions

	/////////////////////
	//Data Transfer (dtf)
	bool		bDtf;			//dtf flag
	bool		bDtfInp;		//inp
	bool		bDtfOut;		//out
	char    	strDtfFileName[16];	//filename
	int 		nAcr;			//access rate [ms]
	fstream		inp;			//file stream
	fstream		out;			//file stream

	bool Dtf(int nMode);		//open, close and check files
	bool DtfRead(float *nnvvalues);				//read from the input file
	bool DtfWrite(long outputcount, float *outputvalues);			//write to the output file

	// Generated message map functions
protected:
//	DECLARE_MESSAGE_MAP();
};

