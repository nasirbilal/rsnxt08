//*!!Sensor,    S1,            leftSonar, sensorSONAR,      ,                    !!*//
//*!!Sensor,    S2,          centreSonar, sensorSONAR,      ,                    !!*//
//*!!Sensor,    S3,           rightSonar, sensorSONAR,      ,                    !!*//
//*!!                                                                            !!*//
//*!!Start automatically generated configuration code.                           !!*//
const tSensors leftSonar            = (tSensors) S1;   //sensorSONAR        //*!!!!*//
const tSensors centreSonar          = (tSensors) S2;   //sensorSONAR        //*!!!!*//
const tSensors rightSonar           = (tSensors) S3;   //sensorSONAR        //*!!!!*//
//*!!CLICK to edit 'wizard' created sensor & motor configuration.                !!*//

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Title:-   Local View
//      Author:-  Lachlan Smith (RobotC)
//                Mark Wakabayashi (Java version)
//                Michael Milford (original algorithim/c code)
//      Purpose:- Implement local view data from sonar sensors
//
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Changes(*)/things to do(-):- (1st Version)
//   -continue testing if cells match
//   -also a way to test wat part of local cell structure is empty to put in new local cells
//    currently done with a counter holding the position of the next empty cell
//   -determine wat angle is appropriate
//   -determine memory load of using float, possibly use float for calcs then multiply by say 10 or 100
//    and store as char or int to save memory
//   -whether to use acos() or just use the dot multipy value for comparison.
//   -size of local cell struct
//   -linking with pose cells
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////
//                       //
//       Header          //
//                       //
///////////////////////////

#include "localNeural.h"

///////////////////////////
//                       //
//       Variable        //
//                       //
///////////////////////////
float localTemp[12]; //holds the temporary neural value
float localComparison[12]; //what the local cell data is loaded into
int rightSonarValue; //obvious
int leftSonarValue;
int centreSonarValue;
float tempCalc = 0; //used for the calcs of the neural networks only used in sensor stuff
char nextEmptyCell = 0; //used for holding the next empty cell in the localcell Struct

///////////////////////////
//                       //
//       Function        //
//                       //
///////////////////////////

void clearTemp()
{
	//clears the temp file
	char x;
  for (x = 0; x<12; x++)
  {
    localTemp[x] = 0;
  }
}

void setRight()
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	rightSonarValue = SensorValue(rightSonar);
	tempCalc = 0;
	if(rightSonarValue <= firstUnit)
	{
    localTemp[0] = (float) (1 - (firstUnit - rightSonarValue)/firstUnit);
	}
  else if(rightSonarValue > firstUnit && rightSonarValue <= secondUnit)
  {
  	tempCalc = (float) (secondUnit - rightSonarValue)/(secondUnit - firstUnit);
    localTemp[0] = tempCalc;
  	localTemp[1] = 1- tempCalc;
  }
  else if(rightSonarValue > secondUnit && rightSonarValue <= thirdUnit)
  {
  	tempCalc = (float) (thirdUnit - rightSonarValue)/(thirdUnit - secondUnit);
    localTemp[1] = tempCalc;
  	localTemp[2] = 1 - tempCalc;
  }
  else if(rightSonarValue > thirdUnit && rightSonarValue <= fourthUnit)
  {
  	tempCalc = (float) (fourthUnit - rightSonarValue)/(fourthUnit - thirdUnit);
    localTemp[2] = tempCalc;
  	localTemp[3] = 1 - tempCalc;
  }
}

void setCentre()
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	centreSonarValue = SensorValue(centreSonar);
	tempCalc = 0;
	if(centreSonarValue <= firstUnit)
	{
    localTemp[4] = (float) (1 - (firstUnit - centreSonarValue)/firstUnit);
	}
  else if(centreSonarValue > firstUnit && centreSonarValue <= secondUnit)
  {
  	tempCalc = (float) (secondUnit - centreSonarValue)/(secondUnit - firstUnit);
    localTemp[4] = tempCalc;
  	localTemp[5] = 1- tempCalc;
  }
  else if(centreSonarValue > secondUnit && centreSonarValue <= thirdUnit)
  {
  	tempCalc = (float) (thirdUnit - centreSonarValue)/(thirdUnit - secondUnit);
    localTemp[5] = tempCalc;
  	localTemp[6] = 1 - tempCalc;
  }
  else if(centreSonarValue > thirdUnit && centreSonarValue <= fourthUnit)
  {
  	tempCalc = (float) (fourthUnit - centreSonarValue)/(fourthUnit - thirdUnit);
    localTemp[6] = tempCalc;
  	localTemp[7] = 1 - tempCalc;
  }
}

void setLeft()
{
	leftSonarValue = SensorValue(leftSonar);
	tempCalc = 0;
	if(leftSonarValue <= firstUnit)
	{
    localTemp[8] = (float) (1 - (firstUnit - leftSonarValue)/firstUnit);
	}
  else if(leftSonarValue > firstUnit && leftSonarValue <= secondUnit)
  {
  	tempCalc = (float) (secondUnit - leftSonarValue)/(secondUnit - firstUnit);
    localTemp[8] = tempCalc;
  	localTemp[9] = 1- tempCalc;
  }
  else if(leftSonarValue > secondUnit && leftSonarValue <= thirdUnit)
  {
  	tempCalc = (float) (thirdUnit - leftSonarValue)/(thirdUnit - secondUnit);
    localTemp[9] = tempCalc;
  	localTemp[10] = 1 - tempCalc;
  }
  else if(leftSonarValue > thirdUnit && leftSonarValue <= fourthUnit)
  {
  	tempCalc = (float) (fourthUnit - leftSonarValue)/(fourthUnit - thirdUnit);
    localTemp[10] = tempCalc;
  	localTemp[11] = 1 - tempCalc;
  }
}
/*
void dotMultiply(float array1[12], float array2[12])
{
  char i;
  float tempArray[12];
  for(i = 0; i < 12; i++)
  {
    tempArray[i] = array1[i] * array2[i];
  }
  localTemp = tempArray;
}
*/

float dotMultiply()
{
  char i;
  float dotValue = 0;
  for(i = 0; i < 12; i++)
  {
    dotValue = dotValue + localTemp[i] * localComparison[i];
  }
  return dotValue;
}

void normaliseTemp()
{
//will normalise the temp ||a|| = sqrt(a1^2 + a2^2 + a3^3)
//normalise by a/||a||
	float tempTotal = 0;
	char y;
	for(y = 0; y < 12; y++)
	{
		tempTotal = tempTotal + localTemp[y]^2;
	}
	tempTotal = sqrt(tempTotal);
	for(y = 0; y < 12; y++)
	{
	  localTemp[y] = localTemp[y]/tempTotal;
	}
}

void setTemp()
{
  clearTemp();
	setRight();
	setCentre();
	setLeft();
	normaliseTemp();
}

void checkLocalCell()
{
	char z;
	float dotTempValue;
	float holderCell[12];
	holderCell = localTemp;
  for(z = 0; z<numLocalCells; z++)
  {
  	if(localCellStruct[0].ACTIVE)
  	{
  	  while(localCellStruct[z].ACTIVE)
  	  {
        localComparison = localCellStruct[z].localCellTemp; //load local cell data
        dotTempValue = dotMultiply(); //dot multiply for comparison
        if(acos(dotTempValue)<50)
        {
          //therefore no match
        	localCellStruct[nextEmptyCell].localCellTemp = holderCell; //new local cell = current data
          localCellStruct[nextEmptyCell].ACTIVE = 1; //make active
          nextEmptyCell++; //increase count for nextEmptyCell
        }
      }
    }
    else {
    localCellStruct[0].localCellTemp = holderCell;
    localCellStruct[0].ACTIVE = 1;
    nextEmptyCell++;
    }
  }


}

task main ()
{
  while(1)
  {
    setTemp();
    nxtDrawLine(5, 20, 94, 20);
    nxtDrawLine(86, 0, 86, 63);
    nxtDrawLine(26, 0, 26, 63);
    nxtDrawLine(56, 0, 56, 63);
    //left
    nxtDisplayStringAt(0, 60, "%1.2f",localTemp[8]);
    nxtDisplayStringAt(0, 50, "%1.2f",localTemp[9]);
    nxtDisplayStringAt(0, 40, "%1.2f",localTemp[10]);
    nxtDisplayStringAt(0, 30, "%1.2f",localTemp[11]);

    //centre
    nxtDisplayStringAt(30, 60, "%1.2f",localTemp[4]);
    nxtDisplayStringAt(30, 50, "%1.2f",localTemp[5]);
    nxtDisplayStringAt(30, 40, "%1.2f",localTemp[6]);
    nxtDisplayStringAt(30, 30, "%1.2f",localTemp[7]);

    //right
    nxtDisplayStringAt(60, 60, "%1.2f",localTemp[0]);
    nxtDisplayStringAt(60, 50, "%1.2f",localTemp[1]);
    nxtDisplayStringAt(60, 40, "%1.2f",localTemp[2]);
    nxtDisplayStringAt(60, 30, "%1.2f",localTemp[3]);

    //TEXT
    nxtDisplayStringAt(88, 60, "N1");
    nxtDisplayStringAt(88, 50, "N2");
    nxtDisplayStringAt(88, 40, "N3");
    nxtDisplayStringAt(88, 30, "N4");
    nxtDisplayStringAt(13, 10, "L");
    nxtDisplayStringAt(40, 10, "C");
    nxtDisplayStringAt(69, 10, "R");
    wait1Msec(1000);

    nxtDisplayCenteredTextLine(1,"Normalised");
    wait1Msec(100);
  }

}
