//*!!Sensor,    S1,            leftSonar, sensorSONAR,      ,                    !!*//
//*!!Sensor,    S2,          centreSonar, sensorSONAR,      ,                    !!*//
//*!!Sensor,    S3,           rightSonar, sensorSONAR,      ,                    !!*//
//*!!Motor,  motorB,            leftMotor, tmotorNxtEncoderClosedLoop,           !!*//
//*!!Motor,  motorC,           rightMotor, tmotorNxtEncoderClosedLoop,           !!*//
//*!!                                                                            !!*//
//*!!Start automatically generated configuration code.                           !!*//
const tSensors leftSonar            = (tSensors) S1;   //sensorSONAR        //*!!!!*//
const tSensors centreSonar          = (tSensors) S2;   //sensorSONAR        //*!!!!*//
const tSensors rightSonar           = (tSensors) S3;   //sensorSONAR        //*!!!!*//
const tMotor   leftMotor            = (tMotor) motorB; //tmotorNxtEncoderClosedLoop //*!!!!*//
const tMotor   rightMotor           = (tMotor) motorC; //tmotorNxtEncoderClosedLoop //*!!!!*//
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
//  Changes(*)/things to do(-)/what works("):- (1st Version)
//   -continue testing if cells match
//   -determine memory load of using float, possibly use float for calcs then multiply by say 10 or 100
//    and store as char or int to save memory
//   -whether to use acos() or just use the dot multiply value for comparison.
//   -size of local cell struct
//   -linking with pose cells
//   "dotMuliply function works as advertised and normalises the neural data.
//   *<0.3 is a match (radians)
//   "Reading and comparing local cells works - just need to tweak the angle value
//   *Now equations will not doing anything when a value of an array = 0 - thus speeding up system
//   *now testing with four units with greater difference between them range ~0-200 cm
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
float localTemp[numNeuralUnits]; //holds the temporary neural value
float localComparison[numNeuralUnits]; //what the local cell data is loaded into
float rightSonarValue = 0; //obvious
float leftSonarValue = 0;
float centreSonarValue = 0;
char nextEmptyCell = 0; //used for holding the next empty cell in the localcell Struct
float tempCalcC = 0;
float tempCalcL = 0;
float tempCalcR = 0;

///////////////////////////
//                       //
//       Function        //
//                       //
///////////////////////////

void clearTemp()
{
  memset(localTemp, 0, 48);
  memset(localComparison, 0, 48);
}

/*
' change to five neural units and change distance between them.
'
*/

void setRight()
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	rightSonarValue = SensorValue(rightSonar);
	tempCalcR = 0;
	if(rightSonarValue <= firstUnit)
	{
		tempCalcR = (float) (((firstUnit - rightSonarValue)/firstUnit));
    localTemp[8] = (float) 1 - tempCalcR;
	}
  else if(rightSonarValue > firstUnit && rightSonarValue <= secondUnit)
  {
  	tempCalcR = (float) (secondUnit - rightSonarValue)/(secondUnit - firstUnit);
    localTemp[8] = tempCalcR;
  	localTemp[9] = 1 - tempCalcR;
  }
  else if(rightSonarValue > secondUnit && rightSonarValue <= thirdUnit)
  {
  	tempCalcR = (float) (thirdUnit - rightSonarValue)/(thirdUnit - secondUnit);
    localTemp[9] = tempCalcR;
  	localTemp[10] = 1 - tempCalcR;
  }
  else if(rightSonarValue > thirdUnit && rightSonarValue <= fourthUnit)
  {
  	tempCalcR = (float) (fourthUnit - rightSonarValue)/(fourthUnit - thirdUnit);
    localTemp[10] = tempCalcR;
  	localTemp[11] = 1 - tempCalcR;
  }
  else if(rightSonarValue > fourthUnit)
  {
    localTemp[11] = 1;
  }
}

void setCentre()
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	centreSonarValue = SensorValue(centreSonar);
	tempCalcC = 0;
	if(centreSonarValue <= firstUnit)
	{
		tempCalcC = (float) ((firstUnit - centreSonarValue)/firstUnit);
    localTemp[4] = (float) 1 - tempCalcC;
	}
  else if(centreSonarValue > firstUnit && centreSonarValue <= secondUnit)
  {
  	tempCalcC = (float) (secondUnit - centreSonarValue)/(secondUnit - firstUnit);
    localTemp[4] = tempCalcC;
  	localTemp[5] = 1 - tempCalcC;
  }
  else if(centreSonarValue > secondUnit && centreSonarValue <= thirdUnit)
  {
  	tempCalcC = (float) (thirdUnit - centreSonarValue)/(thirdUnit - secondUnit);
    localTemp[5] = tempCalcC;
  	localTemp[6] = 1 - tempCalcC;
  }
  else if(centreSonarValue > thirdUnit && centreSonarValue <= fourthUnit)
  {
  	tempCalcC = (float) (fourthUnit - centreSonarValue)/(fourthUnit - thirdUnit);
    localTemp[6] = tempCalcC;
  	localTemp[7] = 1 - tempCalcC;
  }
  else if(centreSonarValue > fourthUnit)
  {
    localTemp[7] = 1;
  }
}

void setLeft()
{
	leftSonarValue = SensorValue(leftSonar);
	tempCalcL = 0;
	if(leftSonarValue <= firstUnit)
	{
		tempCalcL = (float) ((firstUnit - leftSonarValue)/firstUnit);
    localTemp[0] = (float) 1 - tempCalcL;
	}
  else if(leftSonarValue > firstUnit && leftSonarValue <= secondUnit)
  {
  	tempCalcL = (float) (secondUnit - leftSonarValue)/(secondUnit - firstUnit);
    localTemp[0] = tempCalcL;
  	localTemp[1] = 1 - tempCalcL;
  }
  else if(leftSonarValue > secondUnit && leftSonarValue <= thirdUnit)
  {
  	tempCalcL = (float) (thirdUnit - leftSonarValue)/(thirdUnit - secondUnit);
    localTemp[1] = tempCalcL;
  	localTemp[2] = 1 - tempCalcL;
  }
  else if(leftSonarValue > thirdUnit && leftSonarValue <= fourthUnit)
  {
  	tempCalcL = (float) (fourthUnit - leftSonarValue)/(fourthUnit - thirdUnit);
    localTemp[2] = tempCalcL;
  	localTemp[3] = 1 - tempCalcL;
  }
  else if(leftSonarValue > fourthUnit)
  {
    localTemp[3] = 1;
  }
}

void fillStructArray(char cellNum)
{
  //due to robotC being a bitch and not passing an array as a whole to another array within a structure
	//i am forced to implement it the slow way - hopefully the next version of robotC fixes this
	char k;
	for(k=0; k<numNeuralUnits; k++)
	{
	  localCellStruct[cellNum].localCellTemp[k] = localTemp[k];

	}
}

float dotMultiply()
{
  char i;
  float dotValue = 0;
  for(i = 0; i < numNeuralUnits; i++)
  {
    if(localTemp[i]>0)
    {
      dotValue = dotValue + localTemp[i] * localComparison[i];
    }
  }
  return dotValue;
}

void normaliseTemp()
{
//will normalise the temp ||a|| = sqrt(a1^2 + a2^2 + a3^3)
//normalise by a/||a||
	float tempTotal = 0;
	char y;
	for(y = 0; y < numNeuralUnits; y++)
	{
		if(localTemp[y]>0)
		{
		  tempTotal = tempTotal + localTemp[y]*localTemp[y];
	  }
	}
	tempTotal = sqrt(tempTotal);
	for(y = 0; y < numNeuralUnits; y++)
	{
		if(localTemp[y]>0)
		{
	    localTemp[y] = (float) localTemp[y]/tempTotal;
	  }
	}
}

void setTemp()
{
	//initalises temp, get sensor readings and sets the local temp to a normalised neural vector
  clearTemp();
	setLeft();
	setCentre();
	setRight();
	normaliseTemp();
}

void checkLocalCell()
{
	//uses a normalised version of the localTemp.  The comparison local cells are stored as normalised when 1st created
	char z;
	char match = 0;
	float dotTempValue = 0;
	float tempAngle = 0;
	if(nextEmptyCell == 0)
	{
	  //first localCellView
		fillStructArray(nextEmptyCell);
    nextEmptyCell++;
    eraseDisplay();
    nxtDisplayCenteredTextLine(3, "New Cell Created");
    wait1Msec(800);
    eraseDisplay();
	}
  else {
    for(z = 0; z<nextEmptyCell; z++)
    {
    	//search for a previous local cell that matches the current view to a certain degree
  	  localComparison[0] = localCellStruct[z].localCellTemp[0];
  	  localComparison[1] = localCellStruct[z].localCellTemp[1];
  	  localComparison[2] = localCellStruct[z].localCellTemp[2];
  	  localComparison[3] = localCellStruct[z].localCellTemp[3];
  	  localComparison[4] = localCellStruct[z].localCellTemp[4];
  	  localComparison[5] = localCellStruct[z].localCellTemp[5];
  	  localComparison[6] = localCellStruct[z].localCellTemp[6];
  	  localComparison[7] = localCellStruct[z].localCellTemp[7];
  	  localComparison[8] = localCellStruct[z].localCellTemp[8];
  	  localComparison[9] = localCellStruct[z].localCellTemp[9];
  	  localComparison[10] = localCellStruct[z].localCellTemp[10];
  	  localComparison[11] = localCellStruct[z].localCellTemp[11];
      dotTempValue = dotMultiply();
      tempAngle = acos(dotTempValue);
      eraseDisplay();
      //nxtDisplayString(5, "%3.3f", tempAngle);
      //wait1Msec(1000);
      if(tempAngle<0.30)
      {
        //a match - need to check
      	match = 1;
      	break;
      }
    }
    if(match == 0)
    {//no match found - create a new local view cell
    	fillStructArray(nextEmptyCell);
    	nextEmptyCell++;
    	eraseDisplay();
      nxtDisplayCenteredTextLine(3, "No Match");
      nxtDisplayCenteredTextLine(4, "New Cell Created");
      wait1Msec(800);
      eraseDisplay();
    }

    else if(match == 1)
    {
      eraseDisplay();
      nxtDisplayCenteredTextLine(3, "Match");
      wait1Msec(800);
      eraseDisplay();
    }

  }
}

void doTurn()
{
//part of the testing reigme of the local cells
	if(centreSonarValue<15)
	{
	  if(leftSonarValue > 15 && rightSonarValue < 15)
	  {
	  	nSyncedTurnRatio = 100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = -50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	    nSyncedTurnRatio = -100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = 50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	  }
		else if(leftSonarValue < 15 && rightSonarValue > 15)
	  {
	  	nSyncedTurnRatio = 100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = -50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	    nSyncedTurnRatio = -100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = -50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	  }
		else if(leftSonarValue < 15 && rightSonarValue < 15)
	  {
	  	nSyncedTurnRatio = 100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = -50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	    nSyncedTurnRatio = -100;
	  	nMotorEncoderTarget[motorB] = 370;
	    motor[motorB] = 50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	  }
	  else
	  {
	  	nSyncedTurnRatio = 100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = -50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	  	nSyncedTurnRatio = -100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = 50;
	    while(nMotorRunState[motorB] != runStateIdle) {}

	  }
  }


}

task main ()
{
	while(nextEmptyCell<20)
	{
		char lastCellNum = nextEmptyCell;
	  nSyncedMotors = synchBC;
	  nSyncedTurnRatio = 95;
    nMotorEncoderTarget[motorB] = 200;
	  motor[motorB] = 70;
    while(nMotorRunState[motorB] != runStateIdle) {}
    setTemp();
    checkLocalCell();
    doTurn();
    if(lastCellNum != nextEmptyCell)
    {
      AddToDatalog(1);
    }
    else
    {
    	AddToDatalog(0);
    }
 // wait1Msec(200);

  }
  SaveNxtDatalog();
  PlaySoundFile("! attention.rso");
  while(bSoundActive) {}
}
