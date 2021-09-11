/*---------------------------------------------------
	FILE:   NeuralNet.h
	AUTHOR:	Andreas Balmer-Durrer
	DATE:   August 4 2001
	FUNCTION: This file is the headerfile used to rebuild
			a neural net and uses info's from the channel
	VERSION: 1.0				 
 ---------------------------------------------------*/

#include <fstream.h>
#include <string.h>
#include <stdio.h>

#ifndef _NEURALNET_H
#define _NEURALNET_H


//-----------------------------------------------------------------------------------------
		/*///// !neu! die drei nötigen Arrays\\\\\*/

	struct area					// kreieren einer Datenstruktur, ein Array mit zwei Stellen
	{
		int ID;
		int CoU;
	};  
	
	struct connection
	{
		int suAI;
		int suUI;
		int tuAI;
		int tuUI;
		double Strength;
	} ; 

	struct unit 
	{
		int AI;
		int UI;
		double Value_old;
		double Value_new;
		double Value_exp;
	};

//------------------------------------------------------------------------------------------
enum 
{
	NTF_Sync,
	NTF_End
};
class NeuralNet
{
public: NeuralNet(char filename[100], bool input); //wieso hier mit 100 Stellen?
	~NeuralNet();

// Impelmentation
public:
	//////////////////////////////////////
	//Fundamental attributes and functions

	/////////////////////////
	//Net + Data Trasfer ntf
	bool	bNet;		//Net flag
	bool	bNtf;	//Nettransfer
//-----------------------------------------------------------
	area	*Area;
	connection	*Connection;
	unit	*Unit;
//---------------------------------------------------------

	char	strNtfFileName[16]; //filename
	int		nAcr;		//access rate [ms]
	fstream	nexp;		//file stream for the recovery of the net
	ifstream weightFile;
	ofstream				 outFile;

	int		AREALINES;
	int		UNITLINES;
	int		CONNECTIONLINES;

	bool Ntf(int nMode);		//open, close and check files
	bool NtfRead();//read from the export file of NnetView
	bool NetCalculate();
	void NtfValueNewtoOld();
	void NtfMotorWrite(float *nvalues);
	void NtfSensorRead(float *svalues);
	void NtfReflexActivate(float *svalues, int i, int q);
	void NtfGAChangeWeights(const float *wvalues);
	void NtfWriteWeightsToGenome();
	void NtfGAWeightstoNeuralNet();

private:
	bool SelectConnection(int l);
	void WritetoFile(int a,int l, int weightInfos []);//if a==0 original info; if a==1 the changed
	void WriteWeightstoFile(int a, double weightInfos []);
	void WriteOriginalConnection();
	void Writing(char outFileName[], int weightInfos[]);
	void WritingWeights(char outFileName[], double weightInfos[]);
};

#endif