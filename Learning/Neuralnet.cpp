/*--------------------------------------------------------
	FILE:   NeuralNet.cpp
	AUTHOR:	Andreas Balmer-Durrer
			modified by Daniel Baumgartner for message
			exchange with learning class
	DATE:   August 17 2001
	FUNCTION: This class contains all information to 
			rebuild a neural net, which is exported
			from NNetview
	VERSION: 1 running reading function 				 
 --------------------------------------------------------*/
#include "NeuralNet.h"
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <stdio.h>
#include "Agent.h"

#ifndef _NEURALNET_CPP
#define _NEURALNET_CPP

extern int EXPFILECOUNT;
extern int AREACOUNT;
extern int CONNECTIONCOUNT;
extern int UNITCOUNT;
extern int MOTORCOUNT;
extern int TICK_BIS_REFLEX_START;
extern int MN_LEFT_KNEE_FLEX;
extern int MN_RIGHT_ANKLE_EXT;
extern int SENSOR_START;
extern int SENSOR_END;

//Constructor
NeuralNet::NeuralNet (char filename[32], bool input) // wie gross muss filename sein ? ist doch gut oder?
{
	Area = new area[AREACOUNT];
	Connection = new connection[CONNECTIONCOUNT];
	Unit = new unit[UNITCOUNT];
	
	bNet= true;
	if (input ==1)
	{
		printf("Creating a NeuralNet\n");
		bNtf = true;
	}
	else
	{
		printf("no NeuralNet will be created\n");
		bNtf = false;
	}

	strcpy(strNtfFileName, filename);
	nAcr			= 25; //[ms]
}

//Destructor
NeuralNet::~NeuralNet()
{
	nexp.close();
}

/////// Neuralnet Transfer Function
bool NeuralNet::Ntf(int nMode)
{
	if (nMode == NTF_End) // falls nMode ==1, soll nexp geschl.
	{
		nexp.close();
		return true;
	}

	if ((bNet)&&(bNtf))
	{
		if (!nexp.is_open())	//check  existence of Nnetview export file
			nexp.open(strNtfFileName, ios::in | ios::nocreate, filebuf::sh_write);
		if (!nexp.is_open()) 
		{
			//create Nnetview export file
			nexp.open(strNtfFileName, ios::in, filebuf::sh_write);
			nexp.close();
			return false;
		}
	}
	return true;
}

//// This function reads in the info values of the nnv-file into the different arrays
bool NeuralNet::NtfRead()
{
	Ntf(NTF_Sync);

	if((bNet)&&(bNtf))
	{
		char *pCh;
		pCh = new char[EXPFILECOUNT];
		
		if (nexp.is_open())
		{
			nexp.seekp(0,ios::beg);
			// Netz wird eingelesen
			nexp.getline(pCh,EXPFILECOUNT);
			
			// read in Area Array 
			if(strncmp(pCh,"[AREA]", 6)==0)
			{
				nexp.getline(pCh,EXPFILECOUNT);
				if(strncmp(pCh,"#'AreaID;Name;Nettype;Count of Units;'",30)==0)
				{
					int x = 0;
					nexp.getline(pCh,EXPFILECOUNT);
					while(!(strncmp(pCh,"[END]",6)==0))
					{
						char *arID_ptr;			// pointer auf Adresse mit Area ID
						char *arName_ptr;		// pointer auf 		"	Name
						char *arNettype_ptr;		// pointer auf 		"	Netztyp
						char *arCountoU_ptr;		//			"	Count of Units
						char *arEnd_ptr;		//			"	Ende der Line			
						char areaID[2];
						char areaCoU[2];

						// Adressen der Zeiger zuweisen und string bearbeiten
						arID_ptr = pCh;		// die Adresse von pCh wird arID_ptr zugewiesen

						arName_ptr = strchr(pCh,';');// die Adresse des ersten ; wird arName_ptr zugew.
						if(arName_ptr == NULL)
						{
							cerr<<"Error: unable to finde first ; in Area  line Nr.:"<<x<<endl;
							return false;
						}
						*arName_ptr = '\0';		// der Wert an dieser Adresse wird gelöscht
						++arName_ptr;// pointer auf nächste Stelle grückt (Beginn des Names)

						arNettype_ptr = strchr(arName_ptr,';');	// die Adresse des 2.; wird arNettype_ptr zugew.
						if(arNettype_ptr == NULL)
						{
							cerr<<"Error: unable to finde second ; in Area line Nr.:"<<x<<endl;
							return false;
						}
						*arNettype_ptr = '\0';		// der Wert an dieser Adresse wird gelöscht
						++arNettype_ptr;		// pointer auf nächste Stelle grückt (Beginn von Nettype)
					
						arCountoU_ptr = strchr(arNettype_ptr,';');	// die Adresse des 3.; wird arCountoU_ptr zugewiesen
						if(arCountoU_ptr == NULL)
						{
							cerr<<"Error: unable to finde third ; in Area line Nr.:"<<x;
							return false;
						}
						*arCountoU_ptr = '\0';		// der Wert an dieser Adresse wird gelöscht
						++arCountoU_ptr;		// pointer auf nächste Stelle grückt (Beginn von Count of Unit)
	
						arEnd_ptr = strchr(arCountoU_ptr,';');	// die Adresse des 4.; wird arEnd_ptr zugew.
						if(arEnd_ptr == NULL)
						{
							cerr<<"Error: unable to finde forth ; in Area line Nr.:"<<x;
							return false;
						}
						*arEnd_ptr = '\0';		// der Wert an dieser Adresse wird gelöscht
						++arEnd_ptr;			// pointer zeigt nun auf Ende
					
						// Inhalt zwischen den Zeiger in Array einlesen
						for (int a=0; a<2; a++) //Sets the array to 0, preparing for the new storage
						{
							areaID[a]=0;
							areaCoU[a]=0;
						}

						int  j = 0;
						while(arID_ptr != arName_ptr-1)
						{
							
							areaID[j] = *arID_ptr;	//Inhalt wird für die Stellen in Array geschrieben
							++arID_ptr;
							++j;
						}
						j=0;
						while(arCountoU_ptr != arEnd_ptr-1)
						{
							areaCoU[j] = *arCountoU_ptr;//Inhalt wird für die Stellen in Array geschrieben
							++arCountoU_ptr;
							++j;
						}
						
						//	 Der Inhalt von area[] wird nun umgewandelt (von char in int) und in Area[] geschrieben.
						Area[x].ID = 0;
						Area[x].CoU = 0;
						Area[x].ID = atoi(areaID);
						Area[x].CoU = atoi(areaCoU); //siehe Buch seite 306ff
					
						nexp.getline(pCh,EXPFILECOUNT);
					   	x++;		    // wird nur benötigt, um Fehlermeldung mit Zeilennummer zu versehen und Line für AreaArray
					}	// end of while loop
					AREALINES = x-1;
				}
				else
				{
					printf("did not start reading in Area info\n");
					return false;
				}
				nexp.getline(pCh,EXPFILECOUNT);
			}	// end of read in of Area Array

			// read in Connection Array
			if(strncmp(pCh,"[CONNECTION]",10)==0)
			{
			    nexp.getline(pCh,EXPFILECOUNT);
								
			    if(strncmp(pCh,"#'su-AreaID;su-UnitID;tu-AreaID;tu-UnitID;Strength;'",12)==0)
			    {	
				    int x = 0;
					nexp.getline(pCh,EXPFILECOUNT);
					
			        while(!(strncmp(pCh,"[END]",5)==0))
			        {
						char *cosuAI_ptr;
				        char *cosuUI_ptr;
						char *cotuAI_ptr;
						char *cotuUI_ptr;
						char *coStrength_ptr;
						char *coEnd_ptr;
						char connectionsuAI[5];
						char connectionsuUI[5];
						char connectiontuAI[5];
						char connectiontuUI[5];
						char connectionStrength[6];

						// Adressen der Zeiger zuweisen und string bearbeiten
						cosuAI_ptr = pCh;
						cosuUI_ptr = strchr(pCh, ';');					
						if (cosuUI_ptr == NULL)					
						{
							cerr << "Error: unable to finde frist ; in Connection line Nr.:" << x;
							return false;
						}
						*cosuUI_ptr = '\0';
						++cosuUI_ptr;
						cotuAI_ptr = strchr(cosuUI_ptr, ';');					
						if (cotuAI_ptr == NULL)					
						{
							cerr << "Error: unable to finde second ; in Connection line Nr.:" << x;
							return false;
						}
						*cotuAI_ptr = '\0';
						++cotuAI_ptr;

						cotuUI_ptr = strchr(cotuAI_ptr, ';');					
						if (cotuUI_ptr == NULL)					
						{
							cerr << "Error: unable to finde third ; in Connection line Nr.:" << x;
							return false;
						}
	  					*cotuUI_ptr = '\0';
						++cotuUI_ptr;

						coStrength_ptr = strchr(cotuUI_ptr, ';');					
						if (coStrength_ptr == NULL)					
						{
							cerr << "Error: unable to finde forth ; in Connection line Nr.:" << x;
							return false;
						}
						*coStrength_ptr = '\0';
						++coStrength_ptr;
	
						coEnd_ptr = strchr(coStrength_ptr, ';');					
						if (coEnd_ptr == NULL)					
						{
							cerr << "Error: unable to finde last ; in Connection line Nr.:" << x;
							return false;
						}
						*coEnd_ptr = '\0';
						++coEnd_ptr;
						// Inhalt zwischen den Zeiger in Array einlesen; zuerst die Arrays mit 0 füllen
						for (int a=0; a<5; a++)
						{
							connectionsuAI[a]=0;
							connectionsuUI[a]=0;
							connectiontuAI[a]=0;
							connectiontuUI[a]=0;
						}

						for(int b=0; b<6;b++)
							connectionStrength[b]=0;

						int j = 0;
						while (cosuAI_ptr != cosuUI_ptr-1)
						{
							connectionsuAI[j] = *cosuAI_ptr;
							++cosuAI_ptr;
							++j;
						}
						j = 0;
						while (cosuUI_ptr != cotuAI_ptr-1)
						{
							connectionsuUI[j] = *cosuUI_ptr;
							++cosuUI_ptr;
							++j;
						}
						j = 0;
	  					while (cotuAI_ptr != cotuUI_ptr)
						{
							connectiontuAI[j] = *cotuAI_ptr;
							++cotuAI_ptr;
							++j;
						}
						j = 0;
						while (cotuUI_ptr != coStrength_ptr-1)
						{
							connectiontuUI[j] = *cotuUI_ptr;
							++cotuUI_ptr;
							++j;
						}
						j = 0;
						while (coStrength_ptr != coEnd_ptr-1)
						{
							connectionStrength[j] = *coStrength_ptr;
							++coStrength_ptr;
							++j;
						}
						//	 Der Inhalt von connection[] wird nun umgewandelt (von char in float) und in Connection[] geschrieben.
						Connection[x].Strength = 0;
						Connection[x].suAI = 0;
						Connection[x].suUI = 0;
						Connection[x].tuAI = 0;
						Connection[x].tuUI = 0;
						Connection[x].suAI = atoi (connectionsuAI);
						Connection[x].suUI = atoi (connectionsuUI);
						Connection[x].tuAI = atoi (connectiontuAI);
						Connection[x].tuUI = atoi (connectiontuUI);
						Connection[x].Strength = atof (connectionStrength);
						
						//überprüfen ob in exportiertem File rekursive Verbindung vorhanden sind.
						if ((Connection[x].suAI == Connection[x].tuAI) && (Connection[x].suUI == Connection[x].tuUI))
							cout<<"Rekursive Verbindung auf einem Unit in Linie: "<<x<<endl;

						x++;
						nexp.getline(pCh,EXPFILECOUNT);
					}
					CONNECTIONLINES = x-1;
	  			}
	  			else
	  			{
					printf("did not start reading in Connection info\n");
					return false;
	  			}
				nexp.getline(pCh,EXPFILECOUNT);			
			}	//end of read in of connection array

			// read in Unit Array
			if(strncmp(pCh,"[UNIT]",6)==0)
			{
				nexp.getline(pCh,EXPFILECOUNT);
				if(strncmp(pCh,"#'AreaID;UnitID;Name;Value;'",27)==0)
				{
					int x = 0;				
					nexp.getline(pCh,EXPFILECOUNT);
					while(!(strncmp(pCh,"[END]",5)==0))
					{
						char *uAI_ptr;
						char *uUI_ptr;
						char *uName_ptr;
						char *uValue_ptr;
						char *uEnd_ptr;
						char unitAI[5];
						char unitUI[5];
						char unitValue[7];
						// Adressen der Zeiger zuweisen und string bearbeiten
						uAI_ptr = pCh;

						uUI_ptr = strchr(pCh,';');

						if (uUI_ptr == NULL)		
						{
							cerr<<"Error: unable to finde first ; in Unit line Nr: "<<x;
							return false;
						}
						*uUI_ptr ='\0';
						++uUI_ptr;
					
						uName_ptr = strchr(uUI_ptr, ';');
						if (uName_ptr == NULL)
						{
							cerr<<"Error: unable to finde third ; in Unit line Nr: "<<x;
							return false;
						}
						*uName_ptr ='\0';
						++uName_ptr;

						uValue_ptr = strchr(uName_ptr, ';');
						if (uValue_ptr == NULL)
						{
							cerr<<"Error: unable to finde forth ; in Unit line Nr: "<<x;
							return false;
						}
						*uValue_ptr ='\0';
						++uValue_ptr;

						uEnd_ptr = strchr(uValue_ptr, ';');
						if (uEnd_ptr == NULL)
						{
							cerr<<"Error: unable to finde fith ; in Unit line Nr: "<<x;
							return false;
						}
						*uEnd_ptr ='\0';
						++uEnd_ptr;
						
						// Inhalt zwischen den Zeiger in Array einlesen
						for(int a=0; a<5; a++)
						{
							unitAI[a]=0;
							unitUI[a]=0;
						}
						for(int b=0; b<7; b++)
							unitValue[b]=0;
					
						int j = 0;
						while(uAI_ptr != uUI_ptr-1)
						{
							unitAI[j] = *uAI_ptr;
							++uAI_ptr;
							++j;
						}
						
						j = 0;
						while (uUI_ptr != uName_ptr-1)
						{
							unitUI[j] = *uUI_ptr;
							++uUI_ptr;
							++j;
						}
						j = 0;
						while (uValue_ptr != uEnd_ptr-1)
						{
							unitValue[j] = *uValue_ptr;
							++uValue_ptr;
							++j;
						}
						
						//	 Der Inhalt von unitXy[] wird nun umgewandelt (von char in float) und in myUnit geschrieben.
						Unit[x].AI = 0;
						Unit[x].UI = 0;
						Unit[x].Value_new = 0.0;
						Unit[x].Value_old = 0.0;
						Unit[x].Value_exp = 0.0;
						Unit[x].AI = atoi (unitAI);
						Unit[x].UI = atoi (unitUI);
						Unit[x].Value_old = atof (unitValue);
						Unit[x].Value_exp = atof (unitValue);

						nexp.getline(pCh,EXPFILECOUNT);
						x++;
					}
					UNITLINES = x-1;
				}
				else
				{
					printf("did not start reading in Unit info");
					return false;
				}
				nexp.getline(pCh,EXPFILECOUNT);
			}	// end of read in Unit Array
			if (nexp.eof())
				return false;
		}
		// Print out Reading-Error
		else
		{
			nexp.open(strNtfFileName, ios::in | ios::nocreate, filebuf::sh_write);
			printf("nexp ist nicht offen");
			return false;
		}
	}
	return true;
} 

//Calculation of the new activity of the net (one timestep)
bool NeuralNet::NetCalculate() 
{
	int s, t;
	
	for(int l=0; l<=CONNECTIONLINES; l++)
	{
		for(s = 0; s <= UNITLINES; s++)
			if((Connection[l].suAI == Unit[s].AI) && (Connection[l].suUI == Unit[s].UI))
				break;
		
		for(t = 0; t <= UNITLINES; t++)
			if((Connection[l].tuAI == Unit[t].AI) && (Connection[l].tuUI == Unit[t].UI))
				break;
		
		Unit[t].Value_new += Unit[s].Value_old * Connection[l].Strength;
	}
	return true;
}

//copies the new calculated activation into the Positoin old (will be used as activation for the new calclation)the position new is set to 0, ajusting of the activity between 0 and 1
void NeuralNet::NtfValueNewtoOld()	
{
	for(int x=0; x<= UNITLINES; x++)
	{
		Unit[x].Value_old = Unit[x].Value_new;

		bool isTarget = false;
		
		for(int l = 0; l < CONNECTIONLINES; l++)
			if(((Unit[x].AI==Connection[l].tuAI)&&(Unit[x].UI==Connection[l].tuUI)))
			{
				isTarget = true;
				break;
			}

		if(!isTarget)
			Unit[x].Value_old= Unit[x].Value_exp;
		else
			Unit[x].Value_new = 0;
		
		if(Unit[x].Value_old == 0)
			Unit[x].Value_old = 0.0;
		else if(Unit[x].Value_old < 0)
			Unit[x].Value_old = 0.0;
		else if(Unit[x].Value_old >= 1)
			Unit[x].Value_old = 1;
	}
}

//writes the motoractivity into the nvalue Array
void NeuralNet::NtfMotorWrite(float *nvalues)	
{
	for (int y=0; y<12;y++)
		nvalues[y]=0.1;
	
	y=0;
	for(int x=MN_LEFT_KNEE_FLEX; x<MN_RIGHT_ANKLE_EXT+1; x++)
	{
		nvalues[y] = 0.0;
		nvalues[y] = Unit[x].Value_old;
		y++;
	}
}

void NeuralNet::NtfSensorRead(float *svalues)
{
	int y = 0;
	for (int x = SENSOR_START; x < SENSOR_END+1; x++)
	{	
		Unit[x].Value_old = svalues[y];
		y++;
	}
}



/////////////////////////////////////////////////////////////////////////////////
// added methods by db
/////////////////////////////////////////////////////////////////////////////////

// this method returns the connection strength
double NeuralNet::getConnectionStrength(int x)
{
	return Connection[x].Strength;
}

// this method sets the new connection strength
void NeuralNet::setConnectionStrength(int x, double d)
{
	Connection[x].Strength = d;
}

// this method returns the ID of the source unit of connection x
int NeuralNet::getSourceUnitID(int x)
{
	return Connection[x].suUI;
}

// this method returns the ID of the source area of connection x
int NeuralNet::getSourceAreaID(int x)
{
	return Connection[x].suAI;
}

// this method returns the ID of the target unit of connection x
int NeuralNet::getTargetUnitID(int x)
{
	return Connection[x].tuUI;
}

// this method returns the activation of the source unit of connection x
double NeuralNet::getSourceActivation(int x)
{
	int i = getSourceUnitID(x);
	int j = 0;
	
	while (Unit[j].UI != i)
		j++;

	return Unit[j].Value_old;
}

// this method returns the activation of the target unit of connection x
double NeuralNet::getTargetActivation(int x)
{
	int i = getTargetUnitID(x);
	int j = 0;
	
	while (Unit[j].UI != i)
		j++;

	return Unit[j].Value_old;
}


#endif