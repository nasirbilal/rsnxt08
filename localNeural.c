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
//   -size of local cell struct
//   -linking with pose cells
//   "dotMuliply function works as advertised and normalises the neural data.
//   *<0.85 is a match (radians)
//   "Reading and comparing local cells works - just need to tweak the angle value
//   *Now equations will not doing anything when a value of an array = 0 - thus speeding up system
//   *now testing with sixth units with greater difference between them range ~0-195 cm
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
  memset(localTemp, 0, 4*numNeuralUnits);
  memset(localComparison, 0, 4*numNeuralUnits);
}

void setRight()
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	rightSonarValue = SensorValue(rightSonar);
	tempCalcR = 0;
	if(rightSonarValue <= firstUnit)
	{
		tempCalcR = (float) (((firstUnit - rightSonarValue)/firstUnit));
    localTemp[12] = (float) 1 - tempCalcR;
	}
  else if(rightSonarValue > firstUnit && rightSonarValue <= secondUnit)
  {
  	tempCalcR = (float) (secondUnit - rightSonarValue)/(secondUnit - firstUnit);
    localTemp[12] = tempCalcR;
  	localTemp[13] = 1 - tempCalcR;
  }
  else if(rightSonarValue > secondUnit && rightSonarValue <= thirdUnit)
  {
  	tempCalcR = (float) (thirdUnit - rightSonarValue)/(thirdUnit - secondUnit);
    localTemp[13] = tempCalcR;
  	localTemp[14] = 1 - tempCalcR;
  }
  else if(rightSonarValue > thirdUnit && rightSonarValue <= fourthUnit)
  {
  	tempCalcR = (float) (fourthUnit - rightSonarValue)/(fourthUnit - thirdUnit);
    localTemp[14] = tempCalcR;
  	localTemp[15] = 1 - tempCalcR;
  }
  else if(rightSonarValue > fourthUnit && rightSonarValue <= fifthUnit)
  {
  	tempCalcR = (float) (fifthUnit - rightSonarValue)/(fifthUnit - fourthUnit);
    localTemp[15] = tempCalcR;
  	localTemp[16] = 1 - tempCalcR;
  }
  else if(rightSonarValue > fifthUnit && rightSonarValue <= sixthUnit)
  {
  	tempCalcR = (float) (sixthUnit - rightSonarValue)/(sixthUnit - fifthUnit);
    localTemp[16] = tempCalcR;
  	localTemp[17] = 1 - tempCalcR;
  }
  else if(rightSonarValue > sixthUnit)
  {
    localTemp[17] = 1;
  }
}

void setCentre()
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	centreSonarValue = SensorValue(centreSonar);
	tempCalcC = 0;
	if(centreSonarValue <= firstUnit)
	{
		tempCalcC = (float) (((firstUnit - centreSonarValue)/firstUnit));
    localTemp[6] = (float) 1 - tempCalcC;
	}
  else if(centreSonarValue > firstUnit && centreSonarValue <= secondUnit)
  {
  	tempCalcC = (float) (secondUnit - centreSonarValue)/(secondUnit - firstUnit);
    localTemp[6] = tempCalcC;
  	localTemp[7] = 1 - tempCalcC;
  }
  else if(centreSonarValue > secondUnit && centreSonarValue <= thirdUnit)
  {
  	tempCalcC = (float) (thirdUnit - centreSonarValue)/(thirdUnit - secondUnit);
    localTemp[7] = tempCalcC;
  	localTemp[8] = 1 - tempCalcC;
  }
  else if(centreSonarValue > thirdUnit && centreSonarValue <= fourthUnit)
  {
  	tempCalcC = (float) (fourthUnit - centreSonarValue)/(fourthUnit - thirdUnit);
    localTemp[8] = tempCalcC;
  	localTemp[9] = 1 - tempCalcC;
  }
  else if(centreSonarValue > fourthUnit && centreSonarValue <= fifthUnit)
  {
  	tempCalcC = (float) (fifthUnit - centreSonarValue)/(fifthUnit - fourthUnit);
    localTemp[9] = tempCalcC;
  	localTemp[10] = 1 - tempCalcC;
  }
  else if(centreSonarValue > fifthUnit && centreSonarValue <= sixthUnit)
  {
  	tempCalcC = (float) (sixthUnit - centreSonarValue)/(sixthUnit - fifthUnit);
    localTemp[10] = tempCalcC;
  	localTemp[11] = 1 - tempCalcC;
  }
  else if(centreSonarValue > sixthUnit)
  {
    localTemp[11] = 1;
  }
}

void setLeft()
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	leftSonarValue = SensorValue(leftSonar);
	tempCalcL = 0;
	if(leftSonarValue <= firstUnit)
	{
		tempCalcL = (float) (((firstUnit - leftSonarValue)/firstUnit));
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
  else if(leftSonarValue > fourthUnit && leftSonarValue <= fifthUnit)
  {
  	tempCalcL = (float) (fifthUnit - leftSonarValue)/(fifthUnit - fourthUnit);
    localTemp[3] = tempCalcL;
  	localTemp[4] = 1 - tempCalcL;
  }
  else if(leftSonarValue > fifthUnit && leftSonarValue <= sixthUnit)
  {
  	tempCalcL = (float) (sixthUnit - leftSonarValue)/(sixthUnit - fifthUnit);
    localTemp[4] = tempCalcL;
  	localTemp[5] = 1 - tempCalcL;
  }
  else if(leftSonarValue > sixthUnit)
  {
    localTemp[5] = 1;
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

void fillLocalComparison(char cellNum)
{
  char k;
  for(k=0;k<numNeuralUnits;k++)
  {
  localComparison[k] = localCellStruct[cellNum].localCellTemp[k];

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

void StarWars()
{
  //        180 = Tempo
  //          5 = Default octave
  //    Quarter = Default note length
  //        10% = Break between notes
  //
  PlayTone(  698,   15); wait1Msec( 167);  // Note(F, Duration(Eighth))
  PlayTone(  698,   15); wait1Msec( 167);  // Note(F, Duration(Eighth))
  PlayTone(  698,   15); wait1Msec( 167);  // Note(F, Duration(Eighth))
  PlayTone(  932,   90); wait1Msec(1000);  // Note(A#, Duration(Half .))
  PlayTone( 1047,   90); wait1Msec(1000);  // Note(F6, Duration(Half .))
  PlayTone(  933,   15); wait1Msec( 167);  // Note(D#6, Duration(Eighth))
  PlayTone(  880,   15); wait1Msec( 167);  // Note(D6, Duration(Eighth))
  PlayTone(  784,   15); wait1Msec( 167);  // Note(C6, Duration(Eighth))
  PlayTone( 1398,   90); wait1Msec(1000);  // Note(A#6, Duration(Half .))
  PlayTone( 1047,   45); wait1Msec( 500);  // Note(F6, Duration(Quarter .))
  PlayTone(  933,   15); wait1Msec( 167);  // Note(D#6, Duration(Eighth))
  PlayTone(  880,   15); wait1Msec( 167);  // Note(D6, Duration(Eighth))
  PlayTone(  784,   15); wait1Msec( 167);  // Note(C6, Duration(Eighth))
  PlayTone( 1398,   90); wait1Msec(1000);  // Note(A#6, Duration(Half .))
  PlayTone( 1047,   45); wait1Msec( 500);  // Note(F6, Duration(Quarter .))
  PlayTone(  933,   15); wait1Msec( 167);  // Note(D#6, Duration(Eighth))
  PlayTone(  880,   15); wait1Msec( 167);  // Note(D6, Duration(Eighth))
  PlayTone(  933,   15); wait1Msec( 167);  // Note(D#6, Duration(Eighth))
  PlayTone(  784,   60); wait1Msec( 667);  // Note(C6, Duration(Half))
  PlayTone(    0,   30); wait1Msec( 333);  // Note(Rest)
  PlayTone(  698,   15); wait1Msec( 167);  // Note(F, Duration(Eighth))
  PlayTone(  698,   15); wait1Msec( 167);  // Note(F, Duration(Eighth))
  PlayTone(  698,   15); wait1Msec( 167);  // Note(F, Duration(Eighth))
  PlayTone(  932,   90); wait1Msec(1000);  // Note(A#, Duration(Half .))
  PlayTone( 1047,   90); wait1Msec(1000);  // Note(F6, Duration(Half .))
  PlayTone(  933,   15); wait1Msec( 167);  // Note(D#6, Duration(Eighth))
  PlayTone(  880,   15); wait1Msec( 167);  // Note(D6, Duration(Eighth))
  PlayTone(  784,   15); wait1Msec( 167);  // Note(C6, Duration(Eighth))
  PlayTone( 1398,   90); wait1Msec(1000);  // Note(A#6, Duration(Half .))
  PlayTone( 1047,   45); wait1Msec( 500);  // Note(F6, Duration(Quarter .))
  PlayTone(  933,   15); wait1Msec( 167);  // Note(D#6, Duration(Eighth))
  PlayTone(  880,   15); wait1Msec( 167);  // Note(D6, Duration(Eighth))
  PlayTone(  784,   15); wait1Msec( 167);  // Note(C6, Duration(Eighth))
  PlayTone( 1398,   90); wait1Msec(1000);  // Note(A#6, Duration(Half .))
  PlayTone( 1047,   45); wait1Msec( 500);  // Note(F6, Duration(Quarter .))
  PlayTone(  933,   15); wait1Msec( 167);  // Note(D#6, Duration(Eighth))
  PlayTone(  880,   15); wait1Msec( 167);  // Note(D6, Duration(Eighth))
  PlayTone(  933,   15); wait1Msec( 167);  // Note(D#6, Duration(Eighth))
  PlayTone(  784,   60); wait1Msec( 667);  // Note(C6, Duration(Half))
  return;
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
    //eraseDisplay();
    //nxtDisplayCenteredTextLine(3, "New Cell Created");
    //wait1Msec(1000);
    //eraseDisplay();
    PlaySoundFile("! Attention.rso");
    while(bSoundActive) {}
	}
  else {
    for(z = 0; z<nextEmptyCell; z++)
    {
    	//search for a previous local cell that matches the current view to a certain degree
  	  fillLocalComparison(z);
      dotTempValue = dotMultiply();
      tempAngle = acos(dotTempValue);
      eraseDisplay();
      nxtDisplayString(5, "%3.3f", tempAngle);
      wait1Msec(1000);
      if(tempAngle<0.17)
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
    //	eraseDisplay();
     // nxtDisplayCenteredTextLine(3, "No Match");
    //  nxtDisplayCenteredTextLine(4, "New Cell Created");
     // wait1Msec(1000);
    //  eraseDisplay();
      PlaySoundFile("! Attention.rso");
      while(bSoundActive) {}
    }

    else if(match == 1)
    {
     // eraseDisplay();
     // nxtDisplayCenteredTextLine(3, "Match");
     // wait1Msec(1000);
     // eraseDisplay();
      PlaySoundFile("Woops.rso");
      while(bSoundActive) {}
    }

  }
}

void doTurn()
{
//part of the testing reigme of the local cells
	if(centreSonarValue<19)
	{
	  if(leftSonarValue > 19 && rightSonarValue < 19)
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
		else if(leftSonarValue < 19 && rightSonarValue > 19)
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
		else if(leftSonarValue < 19 && rightSonarValue < 19)
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
	nxtDisplayCenteredTextLine(3, "Roaming");
	nxtDisplayCenteredTextLine(5, "This is a test");
	while(nextEmptyCell<numLocalCells)
	{
		alive();
		char lastCellNum = nextEmptyCell;
	  nSyncedMotors = synchBC;
	  nSyncedTurnRatio = 101;
    nMotorEncoderTarget[motorB] = 200;
	  motor[motorB] =60;
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
  StarWars();
}
