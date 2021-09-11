/*--------------------------------------------------------
	FILE:   NeuralNet.cpp
	AUTHOR:	Andreas Balmer-Durrer
	DATE:   August 17 2001
	FUNCTION: This class contains all information to 
			rebuild a neural net, which is exported
			from NNetview
	VERSION: 1 running reading function 				 
 --------------------------------------------------------*/
#include "neuralNet.h"
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <stdio.h>
#include "agent.h"
#include "simParams.h"

#ifndef _NEURALNET_CPP
#define _NEURALNET_CPP

extern	   SIM_PARAMS *simParams;
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
extern int GA_WEIGHTSCOUNT;
extern int REFLEX_TIME;
extern float REFLEX_STIMULUS;
extern int WEIGTHS_INFOS;
extern int WRITE_ORIGINAL_CONNECTION;

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
	delete[] Area;
	delete[] Connection;
	delete[] Unit;
	nexp.close();
}

////////////////////////////////////////////////
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
		//input
		if (!nexp.is_open())
		{ 
			//check  existence of Nneview export file
			nexp.open(strNtfFileName, ios::in | ios::nocreate, filebuf::sh_write);
			cout<<"nexp wird geöffnet";
		}
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

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//// This function reads in the info values of the nnv-file into the different arrays
////
////
bool NeuralNet::NtfRead()
{
	Ntf(NTF_Sync);

	if((bNet)&&(bNtf))
	{
		char *pCh;

		pCh = new char[EXPFILECOUNT];
		
		if (nexp.is_open())
		{
//			cout<<"bin hier!!" << pCh << ";"<<EXPFILECOUNT<<endl;
			nexp.seekp(0,ios::beg);
			
		
////////////////////////////////////////////////////////////
// Netz wird eingelesen
			nexp.getline(pCh,EXPFILECOUNT);//sizeof(pCh)
//			cout<<"bin hier 1" <<pCh<<";"<<";"<<EXPFILECOUNT<< endl;
			
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// read in Area Array 
////////////////////////////////////////////////////////////
			if(strncmp(pCh,"[AREA]", 6)==0)
			{
				
				nexp.getline(pCh,EXPFILECOUNT);
				if(strncmp(pCh,"#'AreaID;Name;Nettype;Count of Units;'",30)==0)
				{
//					cout<<"bin hier 2" << pCh <<endl;
					int x = 0;
					nexp.getline(pCh,EXPFILECOUNT);
					while(!(strncmp(pCh,"[END]",6)==0))
					{
//						cout<<"bin hier 3" << pCh <<endl;
						char *arID_ptr;			// pointer auf Adresse mit Area ID
						char *arName_ptr;		// pointer auf 		"	Name
						char *arNettype_ptr;		// pointer auf 		"	Netztyp
						char *arCountoU_ptr;		//			"	Count of Units
						char *arEnd_ptr;		//			"	Ende der Line			
						char areaID[2];
						char areaCoU[2];

///////////////////////////////////////////////////////
/// Adressen der Zeiger zuweisen und string bearbeiten
						arID_ptr = pCh;		// die Adresse von pCh wird arID_ptr zugewiesen

//						printf ("found arID at %d\n",arID_ptr);

						arName_ptr = strchr(pCh,';');// die Adresse des ersten ; wird arName_ptr zugew.
						if(arName_ptr == NULL)
						{
							cerr<<"Error: unable to finde first ; in Area  line Nr.:"<<x<<endl;
							return false;
						}
						*arName_ptr = '\0';		// der Wert an dieser Adresse wird gelöscht
						++arName_ptr;// pointer auf nächste Stelle grückt (Beginn des Names)

//						printf ("found arName at %d\n",arName_ptr);

						arNettype_ptr = strchr(arName_ptr,';');	// die Adresse des 2.; wird arNettype_ptr zugew.
						if(arNettype_ptr == NULL)
						{
							cerr<<"Error: unable to finde second ; in Area line Nr.:"<<x<<endl;
							return false;
						}
						*arNettype_ptr = '\0';		// der Wert an dieser Adresse wird gelöscht
						++arNettype_ptr;		// pointer auf nächste Stelle grückt (Beginn von Nettype)
					
//						printf ("found arNettype at %d\n",arNettype_ptr);


						arCountoU_ptr = strchr(arNettype_ptr,';');	// die Adresse des 3.; wird arCountoU_ptr zugewiesen
						if(arCountoU_ptr == NULL)
						{
							cerr<<"Error: unable to finde third ; in Area line Nr.:"<<x;
							return false;
						}
						*arCountoU_ptr = '\0';		// der Wert an dieser Adresse wird gelöscht
						++arCountoU_ptr;		// pointer auf nächste Stelle grückt (Beginn von Count of Unit)
	
//						printf ("found arCountoU at %d\n",arCountoU_ptr);
	
						arEnd_ptr = strchr(arCountoU_ptr,';');	// die Adresse des 4.; wird arEnd_ptr zugew.
						if(arEnd_ptr == NULL)
						{
							cerr<<"Error: unable to finde forth ; in Area line Nr.:"<<x;
							return false;
						}
						*arEnd_ptr = '\0';		// der Wert an dieser Adresse wird gelöscht
						++arEnd_ptr;			// pointer zeigt nun auf Ende

//						printf ("found arEnd at %d\n",arEnd_ptr);
					
//////////////////////////////////////////////////////
/// Inhalt zwischen den Zeiger in Array einlesen
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
//							cout<<j<<endl;
						}
						j=0;
						while(arCountoU_ptr != arEnd_ptr-1)
						{
							areaCoU[j] = *arCountoU_ptr;//Inhalt wird für die Stellen in Array geschrieben
							++arCountoU_ptr;
							++j;
//							cout<<j<<endl;
						}
						
//////////////////////////////////////////////////////
///	 Der Inhalt von area[] wird nun umgewandelt (von char in int) und in Area[] geschrieben.
					
						Area[x].ID = 0;
						Area[x].CoU = 0;
						Area[x].ID = atoi(areaID);
						Area[x].CoU = atoi(areaCoU); //siehe Buch seite 306ff
					
//						cout<<atoi (areaID)<<endl;
//						cout<<atoi (areaCoU)<<endl;
	
						nexp.getline(pCh,EXPFILECOUNT);
					   	x++;		    // wird nur benötigt, um Fehlermeldung mit Zeilennummer zu versehen und Line für AreaArray
//						cout<<"Area-Daten wurden eingelesen in Linie:"<< x <<";"<<Area[x].ID <<";"<<Area[x].CoU<<endl; 
						/*delete arID_ptr;
						delete arName_ptr;
						delete arNettype_ptr;
						delete arCountoU_ptr;
						delete arEnd_ptr;*/
						
					}

					AREALINES = x-1;
//					cout<<"x:"<<x<<endl;
					
				
				}
				else
				{
					printf("did not start reading in Area info\n");
					return false;
				}

				nexp.getline(pCh,EXPFILECOUNT);
//				cout<<"bin am Ende von Area:"<<pCh<<endl;
			}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// read in Connection Array
///////////////////////////////////////////////////////////////////////
			if(strncmp(pCh,"[CONNECTION]",10)==0)
			{
//				cout<<"ich bin am Anfang von Conn"<<endl;
			    nexp.getline(pCh,EXPFILECOUNT);
								
			    if(strncmp(pCh,"#'su-AreaID;su-UnitID;tu-AreaID;tu-UnitID;Strength;'",12)==0)
			    {	
//					cout<<"bin am Anfang von Connection"<<pCh<<endl;
				    int x = 0;
					nexp.getline(pCh,EXPFILECOUNT);
//					cout<<"bin am Anfang von Connection"<<pCh<<endl;
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
						


///////////////////////////////////////////////////////
/// Adressen der Zeiger zuweisen und string bearbeiten
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
///////////////////////////////////////////////////////
/// Inhalt zwischen den Zeiger in Array einlesen; zuerst die Arrays mit 0 füllen
						for (int a=0; a<5; a++)
						{
							connectionsuAI[a]=0;
							connectionsuUI[a]=0;
							connectiontuAI[a]=0;
							connectiontuUI[a]=0;
						}

						for(int b=0; b<6;b++)
						{
							connectionStrength[b]=0;
						}

						int j = 0;

						//cout<<"ich bin am Anfang von Conn5"<<endl;
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
							//cout<<"ich bin am Anfang von Conn7"<<endl;
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
///////////////////////////////////////////////////////
///	 Der Inhalt von connection[] wird nun umgewandelt (von char in float) und in Connection[] geschrieben.
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
/////////////////////////////////////////////////////////////////////////
// überprüfen, ob im exportierten File rekursive Bindungen vorhanden sind

						if ((Connection[x].suAI == Connection[x].tuAI) && (Connection[x].suUI == Connection[x].tuUI))
						{
							cout<<"Rekursive Verbindung auf einem Unit in Linie: "<<x<<endl;
						}

						//cout<<"suAI"<<atof(connectionsuAI)<<endl;
						//cout<<"suUI"<<atof(connectionsuUI)<<endl;
						//cout<<"tuAI"<<atof(connectiontuAI)<<endl;
						//cout<<"tuUI"<<atof(connectiontuUI)<<endl;
						//cout<<"Strength"<<atof(connectionStrength)<<endl;
						//printf ("die Unit ID von Connection[%d",x);
						//printf("] ist%d\n",Connection[x].suUI);

						x++;
						nexp.getline(pCh,EXPFILECOUNT);
						/*delete[] cosuAI_ptr;
						delete[] cosuUI_ptr;
						delete[] cotuAI_ptr;
						delete[] cotuUI_ptr;
						delete[] coStrength_ptr;
						delete[] coEnd_ptr;*/
					}

					CONNECTIONLINES = x-1;
				
	  			}
	  			else
	  			{
					printf("did not start reading in Connection info\n");
					return false;
	  			}
			
		  	nexp.getline(pCh,EXPFILECOUNT);			
			}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// read in Unit Array
////////////////////////////////////////////////////////
			if(strncmp(pCh,"[UNIT]",6)==0)
			{
//				cout<<"bin hier 6" << pCh <<endl;
				nexp.getline(pCh,EXPFILECOUNT);
				if(strncmp(pCh,"#'AreaID;UnitID;Name;Value;'",27)==0)
				{
					//cout<<"bin hier 7" << pCh <<endl;
					int x = 0;				
					nexp.getline(pCh,EXPFILECOUNT);
					while(!(strncmp(pCh,"[END]",5)==0))
					{
//						cout<<"bin hier 8" << pCh <<";Line"<<x<<endl;
						char *uAI_ptr;
						char *uUI_ptr;
						char *uName_ptr;
						char *uValue_ptr;
						char *uEnd_ptr;
						char unitAI[5];
						char unitUI[5];
						char unitValue[7];
///////////////////////////////////////////////////////
/// Adressen der Zeiger zuweisen und string bearbeiten
						uAI_ptr = pCh;

						//printf ("found uAI at Position:%d\n",uAI_ptr);
												
						uUI_ptr = strchr(pCh,';');

						//printf ("found UI at %d\n",uUI_ptr);
						

						if (uUI_ptr == NULL)		

						{
							//cout<<"in uUI_ptr steht nicht NULL";
							cerr<<"Error: unable to finde first ; in Unit line Nr: "<<x;
							return false;
						}
						*uUI_ptr ='\0';
						++uUI_ptr;
						//printf ("found UI at %d\n",uUI_ptr);
					
						uName_ptr = strchr(uUI_ptr, ';');
						if (uName_ptr == NULL)
						{
							cerr<<"Error: unable to finde third ; in Unit line Nr: "<<x;
							return false;
						}
						*uName_ptr ='\0';
						++uName_ptr;
						//printf ("found Name at %d\n",uName_ptr);

						uValue_ptr = strchr(uName_ptr, ';');
						if (uValue_ptr == NULL)
						{
							cerr<<"Error: unable to finde forth ; in Unit line Nr: "<<x;
							return false;
						}
						*uValue_ptr ='\0';
						++uValue_ptr;
						//printf ("found Value at %d\n",uValue_ptr);
						//cout<<"mit dem String: "<<uValue_ptr<<endl;

						uEnd_ptr = strchr(uValue_ptr, ';');
						if (uEnd_ptr == NULL)
						{
							cerr<<"Error: unable to finde fith ; in Unit line Nr: "<<x;
							return false;
						}
						*uEnd_ptr ='\0';
						++uEnd_ptr;
						//printf ("found End at %d\n",uEnd_ptr);
						
//////////////////////////////////////////////////////
/// Inhalt zwischen den Zeiger in Array einlesen

						for(int a=0; a<5; a++)
						{
							unitAI[a]=0;
							unitUI[a]=0;
						}
						for(int b=0; b<7; b++)
						{
							unitValue[b]=0;
						}
					
						int j = 0;
						while(uAI_ptr != uUI_ptr-1)
						{
							//cout<<"bin hier 10"<<endl;
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
						

//////////////////////////////////////////////////////
///	 Der Inhalt von unitXy[] wird nun umgewandelt (von char in float) und in myUnit geschrieben.

						Unit[x].AI = 0;
						Unit[x].UI = 0;
						Unit[x].Value_new = 0.0;
						Unit[x].Value_old = 0.0;
						Unit[x].Value_exp = 0.0;
						Unit[x].AI = atoi (unitAI);
						Unit[x].UI = atoi (unitUI);
						Unit[x].Value_old = atof (unitValue);
						//Unit[x].Value_new = atof (unitValue);
						Unit[x].Value_exp = atof (unitValue);

						//cout<<"Die Unit AI ist:"<<atof(unitAI)<<endl;
						//cout<<"Die Unit UI ist:"<<atof(unitUI)<<endl;
						//cout<<"Die Unit Value ist:"<<atof(unitValue)<<endl;					
						nexp.getline(pCh,EXPFILECOUNT);
						x++;
						/*delete[] uAI_ptr;
						delete[] uUI_ptr;
						delete[] uName_ptr;
						delete[] uValue_ptr;
						deete[] uEnd_ptr;*/
						}
					
					UNITLINES = x-1;
				}
				else
				{
					printf("did not start reading in Unit info");
					return false;
				}
				nexp.getline(pCh,EXPFILECOUNT);
				cout<<"Net has been rebuilt"<<endl;
			}
			if (WRITE_ORIGINAL_CONNECTION)
				WriteOriginalConnection();

			if (nexp.eof())
			{
				return false;	
			}
//////////////////////////////////////////////////////////////
/// Print out Reading-Error
			}
			else
			{
				nexp.open(strNtfFileName, ios::in | ios::nocreate, filebuf::sh_write);
				printf("nexp ist nicht offen");
				return false;
			}
						
		}
		return true;
	
} 

///////////////////////////////////////////////////////////////////////
//Calculation of the new activity of the net (one timestep)
///////////////////////////////////////////////////////////////////////
bool NeuralNet::NetCalculate() 
{
	int s;
	int t;
	
	//cout<<CONNECTIONLINES<<"&"<<UNITLINES<<endl;
	for(int l=0; l<=CONNECTIONLINES; l++)
	{
		
		for(s = 0; s <= UNITLINES; s++)
		{
		//	cout<<l<<r;
			if((Connection[l].suAI == Unit[s].AI) && (Connection[l].suUI == Unit[s].UI))
				break;
		}
		
		
		for(t = 0; t <= UNITLINES; t++)
		{
		//	cout<<y<<endl;
			if((Connection[l].tuAI == Unit[t].AI) && (Connection[l].tuUI == Unit[t].UI))
				break;
		}
		
		Unit[t].Value_new += Unit[s].Value_old * Connection[l].Strength;
						 

	}
	

	return true;
}

/////////////////////////////////////////////////////////////////////////
//copys the new calculated activation into the Positoin old (will be used as activation for the new calclation)the position new is set to 0, ajusting of the activity between 0 and 1
/////////////////////////////////////////////////////////////////////////
void NeuralNet::NtfValueNewtoOld()	
{

	for(int x=0; x<= UNITLINES; x++)
	{
		Unit[x].Value_old = Unit[x].Value_new;
		//if(Unit[x].Value_old>0)
			//cout<<"Value_old im ValueNewToOld:"<<Unit[x].Value_old<<"; row #: "<<x<<endl;

		bool isTarget = false;
		
		for(int l = 0; l < CONNECTIONLINES; l++)
		{
			if(((Unit[x].AI==Connection[l].tuAI)&&(Unit[x].UI==Connection[l].tuUI)))
			{
				isTarget = true;
				break;
			}
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

//////////////////////////////////////////////////////////////////////////
//writes the motoractivity into the nvalue Array
//////////////////////////////////////////////////////////////////////////

void NeuralNet::NtfMotorWrite(float *nvalues)	
{
	for (int y=0; y<12;y++)
	{
		nvalues[y]=0.1;
	} 
	y=0;
	for(int x=MN_LEFT_KNEE_FLEX; x<MN_RIGHT_ANKLE_EXT+1; x++)
	{
		nvalues[y] = 0.0;
		nvalues[y] = Unit[x].Value_old;
/*	if ((y==3)||(y==5)//)
			nvalues[y]=1;
		else
			nvalues[11]=0;*/
			
		y++;
//		cout<<"die Motoren werden angesteuert"<<nvalues[y-1]<<";"<<y<<";"<<Unit[x].Value_old<<endl;
	}

}

//-------------------------------------------------------------------------------------
// writes the sensor activation of the biped into the sensorunit of the net
//-------------------------------------------------------------------------------------
void NeuralNet::NtfSensorRead(float *svalues)
{
	int y =0;
	for (int x=SENSOR_START; x<SENSOR_END+1; x++)
	{	
		
		Unit[x].Value_old = svalues[y];
//		cout<<"Value_old ="<<svalues[y]<<";Y="<<y;
	
		y++;	
	}
}
//------------------------------------------------------------------------------
// enables to test reflexes generated by the net (virtual activation of a sensorunit)
//------------------------------------------------------------------------------
void NeuralNet::NtfReflexActivate(float *svalues,int i, int q)
{
	if (q<TICK_BIS_REFLEX_START)
		svalues[i]=0.0;
	else if ((q>=TICK_BIS_REFLEX_START)&&(q<TICK_BIS_REFLEX_START+REFLEX_TIME))
		svalues[i]=REFLEX_STIMULUS;
	else if (q>=TICK_BIS_REFLEX_START+REFLEX_TIME)
		svalues[i]=0.0;
}
//--------------------------------------------------------------------------------
//takes the numbers created by the GA and copys them into the weight array
//--------------------------------------------------------------------------------
void NeuralNet::NtfGAChangeWeights(const float *wvalues)
{
	int g=0;

	int weightsUnit[700];
	int weightsArea[700];
	int weightLine[700];
	int weighttUnit[700];
	int weighttArea[700];
	double weightsValue[700];

	for (int l=0; l<CONNECTIONCOUNT;l++)
	{		
		if(SelectConnection(l))
		{
//			cout<<"Connection[l].Strength: "<<l<<" ; "<<Connection[l].Strength<<endl;
			Connection[l].Strength = wvalues[g];
//			cout<<l<<" ; "<<wvalues[g]<<" ; "<<g<<endl;
			g++;
		}
		if(WEIGTHS_INFOS)
		{
			weightsUnit[l]=Connection[l].suUI;
			weightsArea[l]=Connection[l].suAI;
			weighttUnit[l]=Connection[l].tuUI;
			weighttArea[l]=Connection[l].tuAI;
			weightLine[l]=g;
			weightsValue[l]=Connection[l].Strength;
		}	

	}

	if(WEIGTHS_INFOS)
	{
		WritetoFile(1,0,weightsArea);
		WritetoFile(1,1,weightsUnit);
		WritetoFile(1,2,weightLine);
		WritetoFile(1,3,weighttArea);
		WritetoFile(1,4,weighttUnit);
		WriteWeightstoFile(1,weightsValue);
	}

}
//-----------------------------------------------------------------------------------
// makes shure that the GA does not change weights between plus and minus
//-----------------------------------------------------------------------------------
void NeuralNet::NtfWriteWeightsToGenome()
{
	float *weights;
	weights = new float[GA_WEIGHTSCOUNT];
	int g=0;
	for(int l=0; l<CONNECTIONCOUNT;l++)
	{
//	cout<<"ich führe diese Funktion aus"<<endl;
		if(SelectConnection(l))
		{
			weights[g] = Connection[l].Strength;
//			cout<<weights[g]<<" , "<<l<<endl;
			simParams->Write(weights, g);
			g++;
		}
	}

	cout<<"GA_WEIGHTSCOUNT must be : "<< g<<endl;
}
//------------------------------------------------------------------------------
// takes the weightarray of an earlier GA run and writes it into the Weight Array 
// of Connection. Is to rerun the whole progr with other fitness function but on an
// already changed level.
//-------------------------------------------------------------------------------
void NeuralNet::NtfGAWeightstoNeuralNet()
{
	char gaName[50];
			
	sprintf(gaName,"data/ga%d.dat",0); //Because it usus that the best result is stored as ga0.dat in data Direct.

	weightFile.open(gaName);

	if(!weightFile.is_open())
	{
		cerr<< "was not able to open ga0.dat file in /data/ "<<endl;
	}
	
	int length;
	weightFile >> length;

	int g = 0;
	for(int l = 0; l<CONNECTIONCOUNT; l++)
	{	
		if(SelectConnection(l)&&(g<length))
		{
//			cout<<"Connection[l].Strength vorher: "<<Connection[l].Strength<<endl;
			weightFile >> Connection[l].Strength;
			g++;
//			cout<<"Connection[l].Strength nachher: "<<Connection[l].Strength<<endl;
		}
	}
	weightFile.close();
}

//--------------------------------------------------------------------------------------------
//Private Methods
//--------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
//to select the connections  that should be changed by the GA
//--------------------------------------------------------------------------------------------
bool NeuralNet::SelectConnection(int l)
{	
	if((Connection[l].suAI>15)&&!(Connection[l].suAI==47))//((Connection[l].tuAI==3)||(Connection[l].tuAI==4)||(Connection[l].tuAI==9)||(Connection[l].tuAI==10)||(Connection[l].tuAI==14)||(Connection[l].tuAI==15))
		return true;
	else
		return false;
}

//--------------------------------------------------------------------------------------
//function to write the changed weights with suAI and suUI into a txt file
//--------------------------------------------------------------------------------------

void NeuralNet::WritetoFile(int V,int ID, int weightInfos[])
{
	char outFileName[50];
	int g;

	if(V==0)
	{
		if (ID==0)
			sprintf(outFileName,"weight/orginal_Gewichts_suAID.txt");
		else if (ID==1)
			sprintf(outFileName,"weight/orginal_Gewichts_suUID.txt");
		else if (ID==2)
			sprintf(outFileName,"weight/orginal_Line_in_Connection.txt");
		else if (ID==3)
			sprintf(outFileName,"weight/orginal_Gewichts_tuAID.txt");
		else if (ID==4)
			sprintf(outFileName,"weight/orginal_Gewichts_tuUID.txt");

		Writing(outFileName,weightInfos);
	}
	else if(V==1)
	{
		if (ID==0)
			sprintf(outFileName,"weight/Gewichts_suAID.txt");
		else if (ID==1)
			sprintf(outFileName,"weight/Gewichts_suUID.txt");
		else if (ID==2)
			sprintf(outFileName,"weight/Line_in_Connection.txt");
		else if (ID==3)
			sprintf(outFileName,"weight/Gewichts_tuAID.txt");
		else if (ID==4)
			sprintf(outFileName,"weight/Gewichts_tuAUID.txt");

		Writing(outFileName,weightInfos);

	}
	else
		cerr<<"error in execution of WritetoFile in neuralnet.cpp"<<endl;

	
}

void NeuralNet::WriteWeightstoFile(int V, double weightInfos[])
{
	char outFileName[50];
	
	if(V==0)
	{
		sprintf(outFileName,"weight/Gewichts_Value_orginal.txt");
		
		WritingWeights(outFileName,weightInfos );
	}

	else if(V==1)
	{
		sprintf(outFileName,"weight/Gewichts_Value_GA.txt");

		WritingWeights(outFileName,weightInfos);
	}

	else if (V==2)
		sprintf(outFileName,"weight/Gewichts_Value_control.txt");		
	else
		cerr<<"error in execution of WriteWeightstoFile in neuralnet.cpp"<<endl;


}
//////////////////////////////////////////////////////////////////////////////////////////
//Copys the info saved in the Connection.x struct into an array, to write it afterwards to a txt file
//////////////////////////////////////////////////////////////////////////////////////////
void NeuralNet::WriteOriginalConnection()
{	
	int weightsUnit[700];
	int weightsArea[700];
	int weightLine[700];
	int weighttUnit[700];
	int weighttArea[700];
	double weightsValue[700];

	for (int l=0; l<CONNECTIONCOUNT;l++)
	{		

		weightsUnit[l]=Connection[l].suUI;
		weightsArea[l]=Connection[l].suAI;
		weighttUnit[l]=Connection[l].tuUI;
		weighttArea[l]=Connection[l].tuAI;
		weightLine[l]=l;
		weightsValue[l]=Connection[l].Strength;
		
	}
	
	WritetoFile(0,0,weightsArea);
	WritetoFile(0,1,weightsUnit);
	WritetoFile(0,2,weightLine);
	WritetoFile(0,3,weighttArea);
	WritetoFile(0,4,weighttUnit);
	WriteWeightstoFile(0,weightsValue);
}

void NeuralNet::Writing(char outFileName[],int weightInfos[])
{
//	char outFileName[50];
	int g;

	outFile.open(outFileName);

	for ( g=0; g<CONNECTIONCOUNT; g++ )
		outFile << weightInfos[g] << "\n";

	outFile.close();
}

void NeuralNet::WritingWeights(char outFileName[], double weightInfos[])
{
//	char outFileName[50];
	int g;

	outFile.open(outFileName);

	for ( g=0; g<CONNECTIONCOUNT; g++ )
		outFile << weightInfos[g] << "\n";

	outFile.close();
}


#endif

