#pragma config(Sensor, S1,     leftSonar,           sensorSONAR)
#pragma config(Sensor, S2,     centreSonar,         sensorSONAR)
#pragma config(Sensor, S3,     rightSonar,          sensorSONAR)
#pragma config(Motor,  motorB,          leftMotor,            tmotorNxtEncoderClosedLoop)
#pragma config(Motor,  motorC,          rightMotor,           tmotorNxtEncoderClosedLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

///////////////////////////////////////////////////////////////////////////////////////////////
//
//      Title:- Local cell testing
//      Author:- Lachlan Smith (RobotC)
//               Mark Wakabayashi (Jave version)
//               Michael Milford (original algorithim/c code)
//      Purpose:- Just for testing how well local view works
//
//
///////////////////////////////////////////////////////////////////////////////////////////////

//----include files----//
#include "localNeural.h";
//testing writing to text files
TFileHandle hFileHandle = 0;
TFileHandle hFileHandle2 = 0;
TFileHandle hFileHandle3 = 0;
TFileIOResult nIoResult;
TFileIOResult nIoResultRead;
int nFileSize = 7000;
int nFileSize2 = 7000;
const string sFileName1 = "local1.dat";
const string sFileName2 = "local2.dat";
int checkCounter = 0;



//----General In-Module Variables----//
int numActive = 0; //number of active cells
int currentTheta = 0; //theta for pose and turns
char currentDirection = 0; //direction for pose and turns
localViewCell localTemp; //holds the temporary neural value
localViewCell localComparison; //what the local cell data is loaded into
char nextEmptyCell = 0; //used for holding the next empty cell in the localcell Struct

void writeLocal()
{
	CloseAllHandles(nIoResult);
	OpenWrite(hFileHandle,nIoResult,sFileName1,nFileSize);
	char x;
  for(x = 0; x<numNeuralUnits; x++)
  {
    WriteFloat(hFileHandle, nIoResult, localTemp.localArray[x]);
  }
  Close(hFileHandle,nIoResult);
  checkCounter++;
}

void swapFiles()
{
	OpenWrite(hFileHandle2,nIoResultRead,sFileName2,nFileSize2);
	OpenRead(hFileHandle,nIoResult,sFileName1,nFileSize);
	float tempVar;
  while(nIoResult != ioRsltEndOfFile)
  {
    ReadFloat(hFileHandle,nIoResult,tempVar);
    if(nIoResult == ioRsltEndOfFile) {break;}
    WriteFloat(hFileHandle2,nIoResultRead,tempVar);
  }
	char x;
  for(x = 0; x<numNeuralUnits; x++)
  {
    WriteFloat(hFileHandle2, nIoResult, localTemp.localArray[x]);
  }
  Close(hFileHandle2,nIoResultRead);
  Close(hFileHandle,nIoResult);
  Delete(sFileName1,nIoResult);
  Rename(sFileName1,nIoResult,sFileName2);
  checkCounter++;
}

//                            //
//----Local View Functions----//
//                            //

//----Initialises the arrays to zero----//
void clearTemp()
{
  memset(localTemp, 0, 4*numNeuralUnits); //sets entire array to zero
  memset(localComparison, 0, 4*numNeuralUnits); //sets entire array to zero
}

void datalogging2(char cellNum)
{
	float time = (float) nPgmTime / 1000;
	int time2 = (int) time;
	AddToDatalog(1, time2);
  AddToDatalog(2,cellNum);
  /*if(nSyncedMotors == synchBC)
  {
    AddToDatalog(3,nMotorEncoder[motorB]);
  }
  else if(nSyncedMotors == synchCB)
  {
  	AddToDatalog(3,nMotorEncoder[motorC]);
  }*/
}


//----Set the right neural units----//
void setRight(float rightSonarValue)
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	//rightSonarValue = SensorValue(rightSonar);
	float tempCalc = 0;
	if(rightSonarValue <= firstUnit)
	{
		tempCalc = (float) (((firstUnit - rightSonarValue)/firstUnit));
    localTemp.localArray[12] = (float) 1 - tempCalc;
	}
  else if(rightSonarValue > firstUnit && rightSonarValue <= secondUnit)
  {
  	tempCalc = (float) (secondUnit - rightSonarValue)/(secondUnit - firstUnit);
    localTemp.localArray[12] = tempCalc;
  	localTemp.localArray[13] = 1 - tempCalc;
  }
  else if(rightSonarValue > secondUnit && rightSonarValue <= thirdUnit)
  {
  	tempCalc = (float) (thirdUnit - rightSonarValue)/(thirdUnit - secondUnit);
    localTemp.localArray[13] = tempCalc;
  	localTemp.localArray[14] = 1 - tempCalc;
  }
  else if(rightSonarValue > thirdUnit && rightSonarValue <= fourthUnit)
  {
  	tempCalc = (float) (fourthUnit - rightSonarValue)/(fourthUnit - thirdUnit);
    localTemp.localArray[14] = tempCalc;
  	localTemp.localArray[15] = 1 - tempCalc;
  }
  else if(rightSonarValue > fourthUnit && rightSonarValue <= fifthUnit)
  {
  	tempCalc = (float) (fifthUnit - rightSonarValue)/(fifthUnit - fourthUnit);
    localTemp.localArray[15] = tempCalc;
  	localTemp.localArray[16] = 1 - tempCalc;
  }
  else if(rightSonarValue > fifthUnit && rightSonarValue <= sixthUnit)
  {
  	tempCalc = (float) (sixthUnit - rightSonarValue)/(sixthUnit - fifthUnit);
    localTemp.localArray[16] = tempCalc;
  	localTemp.localArray[17] = 1 - tempCalc;
  }
  else if(rightSonarValue > sixthUnit) //if greater than 195cm
  {
    localTemp.localArray[17] = 1;
  }
}

//----Set centre neural units----//
void setCentre(float centreSonarValue)
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	//centreSonarValue = SensorValue(centreSonar);
	float tempCalc = 0;
	if(centreSonarValue <= firstUnit)
	{
		tempCalc = (float) (((firstUnit - centreSonarValue)/firstUnit));
    localTemp.localArray[6] = (float) 1 - tempCalc;
	}
  else if(centreSonarValue > firstUnit && centreSonarValue <= secondUnit)
  {
  	tempCalc = (float) (secondUnit - centreSonarValue)/(secondUnit - firstUnit);
    localTemp.localArray[6] = tempCalc;
  	localTemp.localArray[7] = 1 - tempCalc;
  }
  else if(centreSonarValue > secondUnit && centreSonarValue <= thirdUnit)
  {
  	tempCalc = (float) (thirdUnit - centreSonarValue)/(thirdUnit - secondUnit);
    localTemp.localArray[7] = tempCalc;
  	localTemp.localArray[8] = 1 - tempCalc;
  }
  else if(centreSonarValue > thirdUnit && centreSonarValue <= fourthUnit)
  {
  	tempCalc = (float) (fourthUnit - centreSonarValue)/(fourthUnit - thirdUnit);
    localTemp.localArray[8] = tempCalc;
  	localTemp.localArray[9] = 1 - tempCalc;
  }
  else if(centreSonarValue > fourthUnit && centreSonarValue <= fifthUnit)
  {
  	tempCalc = (float) (fifthUnit - centreSonarValue)/(fifthUnit - fourthUnit);
    localTemp.localArray[9] = tempCalc;
  	localTemp.localArray[10] = 1 - tempCalc;
  }
  else if(centreSonarValue > fifthUnit && centreSonarValue <= sixthUnit)
  {
  	tempCalc = (float) (sixthUnit - centreSonarValue)/(sixthUnit - fifthUnit);
    localTemp.localArray[10] = tempCalc;
  	localTemp.localArray[11] = 1 - tempCalc;
  }
  else if(centreSonarValue > sixthUnit)
  {
    localTemp.localArray[11] = 1;
  }
}

//----set left neural units----//
void setLeft(float leftSonarValue)
{
	//This function allocates a proportion to each neural unit based on the value returned by the sonar sensor
	//leftSonarValue = SensorValue(leftSonar);
	float tempCalc = 0;
	if(leftSonarValue <= firstUnit)
	{
		tempCalc = (float) (((firstUnit - leftSonarValue)/firstUnit));
    localTemp.localArray[0] = (float) 1 - tempCalc;
	}
  else if(leftSonarValue > firstUnit && leftSonarValue <= secondUnit)
  {
  	tempCalc = (float) (secondUnit - leftSonarValue)/(secondUnit - firstUnit);
    localTemp.localArray[0] = tempCalc;
  	localTemp.localArray[1] = 1 - tempCalc;
  }
  else if(leftSonarValue > secondUnit && leftSonarValue <= thirdUnit)
  {
  	tempCalc = (float) (thirdUnit - leftSonarValue)/(thirdUnit - secondUnit);
    localTemp.localArray[1] = tempCalc;
  	localTemp.localArray[2] = 1 - tempCalc;
  }
  else if(leftSonarValue > thirdUnit && leftSonarValue <= fourthUnit)
  {
  	tempCalc = (float) (fourthUnit - leftSonarValue)/(fourthUnit - thirdUnit);
    localTemp.localArray[2] = tempCalc;
  	localTemp.localArray[3] = 1 - tempCalc;
  }
  else if(leftSonarValue > fourthUnit && leftSonarValue <= fifthUnit)
  {
  	tempCalc = (float) (fifthUnit - leftSonarValue)/(fifthUnit - fourthUnit);
    localTemp.localArray[3] = tempCalc;
  	localTemp.localArray[4] = 1 - tempCalc;
  }
  else if(leftSonarValue > fifthUnit && leftSonarValue <= sixthUnit)
  {
  	tempCalc = (float) (sixthUnit - leftSonarValue)/(sixthUnit - fifthUnit);
    localTemp.localArray[4] = tempCalc;
  	localTemp.localArray[5] = 1 - tempCalc;
  }
  else if(leftSonarValue > sixthUnit)
  {
    localTemp.localArray[5] = 1;
  }
}

//---adds the association between local view and pose----//
void addAssociation()
{
  //creates an association between local view and pose - currently using max activated cell as the one needed but may end
	//up using estimated pose as in RAMP code

	writeLocal();
	/*char k;
	for(k=0; k<numNeuralUnits; k++)
	{
	  poseAssoc[cellNum].localView[k] = localTemp[k];
	}*/
}

//----Dot multiply two vectors----//
float dotMultiply()
{
  char i;
  float dotValue = 0;
  for(i = 0; i < numNeuralUnits; i++)
  {
    if(localTemp.localArray[i]>0)
    {
      dotValue = dotValue + localTemp.localArray[i] * localComparison.localArray[i];
    }
  }
  return dotValue; //return the multiply
}

//----normalise the current view for processing----//
void normaliseTemp()
{
//will normalise the temp ||a|| = sqrt(a1^2 + a2^2 + a3^3)
//normalise by a/||a||
	float tempTotal = 0;
	char y;
	for(y = 0; y < numNeuralUnits; y++)
	{
		if(localTemp.localArray[y]>0) //only if over zero spend clock cycles
		{
		  tempTotal = tempTotal + localTemp.localArray[y]*localTemp.localArray[y]; //a.a
	  }
	}
	tempTotal = sqrt(tempTotal); //sqrt(a.a)
	for(y = 0; y < numNeuralUnits; y++)
	{
		if(localTemp.localArray[y]>0)
		{
	    localTemp.localArray[y] = (float) localTemp.localArray[y]/tempTotal;
	  }
	}
}

//----Gets local view and processes ready for comparison----//
void setTemp()
{
	//initalises temp, get sensor readings and sets the local temp to a normalised neural vector
  clearTemp();
  float rightSonarValue = SensorValue(rightSonar); //obvious
  float leftSonarValue = SensorValue(leftSonar);
  float centreSonarValue = SensorValue(centreSonar);
  setLeft(leftSonarValue);
	setCentre(centreSonarValue);
	setRight(rightSonarValue);
	normaliseTemp();
}

//----Checks if there is a match, add activities, else creates a new association----//
void checkLocalCell()
{
	//uses a normalised version of the localTemp.  The comparison local cells are stored as normalised when 1st created
	char z;
	char breakSymbol =0;
	signed char cellCount = 0;
	char match = 0;
	float dotTempValue = 0;
	float tempAngle = 0;
	if(nextEmptyCell == 0)
	{
	  //first localCellView
		writeLocal();
		datalogging2(nextEmptyCell);
    nextEmptyCell++;

   // PlaySound(soundFastUpwardTones);
    //while(bSoundActive) {}
	}
  else {
  OpenRead(hFileHandle3,nIoResultRead,sFileName1,nFileSize);
  while(nIoResultRead != ioRsltEndOfFile)
    {
    	//search for a previous local cell that matches the current view to a certain degree
    	for(z = 0; z<numNeuralUnits; z++)
    	{
    		ReadFloat(hFileHandle3,nIoResultRead,localComparison.localArray[z]);
    		if(nIoResultRead == ioRsltEndOfFile) {breakSymbol = 1;break;}

    	}
  //  	hFileHandle3 = 0;
    	if(breakSymbol == 1) {break;}
      dotTempValue = dotMultiply();
      tempAngle = acos(dotTempValue);
      if(tempAngle<0.17)//0.17 //if difference less than 10 degrees between vectors
      {
      	datalogging2(cellCount);
      	match = 1;
      	Close(hFileHandle3,nIoResultRead);
      	break; //save cycles break out
      }
      cellCount++;
    }
    if(match == 0)
    {//no match found - create a new local view cell
    Close(hFileHandle3,nIoResultRead);
    swapFiles();
    	datalogging2(cellCount);
    	nextEmptyCell++;
    	nxtDisplayCenteredTextLine(6, "cell created");
      //PlaySound(soundFastUpwardTones);
      //while(bSoundActive) {}
    }
    else if(match == 1)
    {
      //PlaySound(soundBeepBeep);
      //while(bSoundActive) {}
    }
  }
}

//----Set direction and theta if turning----//
void handleDirection(char turn)
{
	switch (turn)
	{
	  case 'l':
	    if(currentDirection == 0)
	    {
	      currentDirection = 1;
	    	currentTheta = 90;
	    }
		  else if(currentDirection == 1)
		  {
		    currentDirection = 2;
		  	currentTheta = 180;
		  }
		  else if(currentDirection == 2)
		  {
		  	currentDirection = 3;
		  	currentTheta = 270;
		  }
		  else if(currentDirection == 3)
		  {
		  	currentDirection = 0;
		  	currentTheta = 0;
		  }
		  break;
		 case 'r':
	    if(currentDirection == 0)
	    {
	      currentDirection = 3;
	    	currentTheta = 270;
	    }
		  else if(currentDirection == 1)
		  {
		    currentDirection = 0;
		  	currentTheta = 0;
		  }
		  else if(currentDirection == 2)
		  {
		  	currentDirection = 1;
		  	currentTheta = 90;
		  }
		  else if(currentDirection == 3)
		  {
		  	currentDirection = 2;
		  	currentTheta = 180;
		  }
		  break;
		  case 'b':
	    if(currentDirection == 0)
	    {
	      currentDirection = 2;
	    	currentTheta = 180;
	    }
		  else if(currentDirection == 1)
		  {
		    currentDirection = 3;
		  	currentTheta = 270;
		  }
		  else if(currentDirection == 2)
		  {
		  	currentDirection = 0;
		  	currentTheta = 0;
		  }
		  else if(currentDirection == 3)
		  {
		  	currentDirection = 1;
		  	currentTheta = 90;
		  }
		  break;
		 default: break;
	}
}


//----Turns if need to----//
void doTurn()
{
//part of the testing reigme of the local cells
	//decided that anticlockwise is positive
	float rightSonarValue = SensorValue(rightSonar); //obvious
  float leftSonarValue = SensorValue(leftSonar);
  float centreSonarValue = SensorValue(centreSonar);


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
	    motor[motorB] = -50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	    handleDirection('l');
	  }
		else if(leftSonarValue < 19 && rightSonarValue > 19)
	  {
	  	nSyncedTurnRatio = 100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = -50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	    nSyncedTurnRatio = -100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = 50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	    handleDirection('r');
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
      handleDirection('b');
	   }
	  else
	  {
	  	nSyncedTurnRatio = 100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = -50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
	  	nSyncedTurnRatio = -100;
	  	nMotorEncoderTarget[motorB] = 190;
	    motor[motorB] = -50;
	    while(nMotorRunState[motorB] != runStateIdle) {}
      handleDirection('l');
	  }
  }
}

//----main----//
task main ()
{
	Delete(sFileName1, nIoResult);
	Delete(sFileName2,nIoResultRead);
	nxtDisplayCenteredTextLine(3, "Roaming");
	nxtDisplayCenteredTextLine(5, "This is a test");
	currentDirection = 0; //set initial
  currentTheta = 0;
  setTemp();  //get local view
  checkLocalCell(); //create first association
  char numLoops = 0;
  int motorBError = 0;
  int motorCError = 0;
  int step1 = 160;
  int step2 = 123;


  while(numLoops < 2)
	{

	nSyncedMotors = synchNone;
  nMotorEncoder[motorB] = 0;
  nMotorEncoder[motorC] = 0;
  motor[motorC] = 0;
	nSyncedMotors = synchBC;
	nSyncedTurnRatio = 60;
	nMotorPIDSpeedCtrl[motorB] = mtrEncoderReg;
	nMotorPIDSpeedCtrl[motorC] = mtrSyncRegMaster;



  nMotorEncoderTarget[motorB] = 3980;
  motor[motorB] = 60;
  while(nMotorRunState[motorB] != runStateIdle){

  	setTemp();
    checkLocalCell();
   // hFileHandle = 0;
 // hFileHandle2 = 0;

   }


  eraseDisplay();
  nxtDisplayCenteredBigTextLine(4, "%d", nMotorEncoder[motorB]);
  nxtDisplayCenteredBigTextLine(2, "%d", nMotorEncoder[motorC]);
  wait10Msec(80);


  nSyncedMotors = synchNone;
  nMotorEncoder[motorB] = 0;
  nMotorEncoder[motorC] = 0;
  motor[motorB] = 0;
  nSyncedMotors = synchCB;
	nSyncedTurnRatio = 60;
  nMotorPIDSpeedCtrl[motorC] = mtrEncoderReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSyncRegMaster;
  nxtDisplayCenteredBigTextLine(4, "%d", nMotorEncoder[motorC]);
  nxtDisplayCenteredBigTextLine(2, "%d", nMotorEncoder[motorC]);
  wait10Msec(80);
  eraseDisplay();

    nMotorEncoderTarget[motorC] = 3840;
  	motor[motorC] = 60;

    while(nMotorRunState[motorC] != runStateIdle)
    {

    	//if((nMotorEncoder[motorB] % 10) == 0)
   // {
  	setTemp();
    checkLocalCell();
   // hFileHandle = 0;
 // hFileHandle2 = 0;
   // };

   }
    nxtDisplayCenteredBigTextLine(3, "%d", nMotorEncoder[motorC]);


  nSyncedMotors = synchNone;
  nMotorEncoder[motorB] = 0;
  nMotorEncoder[motorC] = 0;
  motor[motorC] = 0;
  numLoops++;
  wait10Msec(20);
		/*
    while(nMotorEncoder[motorB]<6560) //7 times round circle
	  {
		  alive(); //stop NXT from sleeping
	    nSyncedMotors = synchBC;
	    nMotorPIDSpeedCtrl[motorB] = mtrEncoderReg;
      nMotorPIDSpeedCtrl[motorC] = mtrSyncRegSlave;
	    nSyncedTurnRatio = 77; //move forward
      nMotorEncoderTarget[motorB] = 160;
	    motor[motorB] =70;
      while(nMotorRunState[motorB] != runStateIdle) {}
      eraseDisplay();
		  nxtDisplayCenteredTextLine(2, "B: %6d",nMotorEncoder[motorB]);
      nxtDisplayCenteredTextLine(3, "C: %6d",nMotorEncoder[motorC]);
      setTemp();
      checkLocalCell();
      motorBError = nMotorEncoder[motorB] - step1;

      motorCError = nMotorEncoder[motorC] - step2;

      nxtDisplayCenteredTextLine(4, "Berr: %6d",motorBError);
      nxtDisplayCenteredTextLine(5, "Cerr: %6d",motorCError);
      //wait10Msec(100);
      nSyncedMotors = synchNone;
      nMotorEncoderTarget[motorB] = motorBError;
      nMotorEncoderTarget[motorC] = motorCError;
      if(motorBError>0)
      {
        motor[motorB] = -15;
      }
      else if(motorBError<0)
      {
      	motor[motorB] = 15;
      }
      if(motorCError>0)
      {
      	motor[motorC] = -15;
      }
      else if(motorCError)
      {
        motor[motorC] = 15;
      }
      while(nMotorRunState[motorB] != runStateIdle || nMotorRunState[motorC] != runStateIdle) {}
      nxtDisplayCenteredTextLine(1, "Fixed");
      motorBError = nMotorEncoder[motorB] - step1;
      motorCError = nMotorEncoder[motorC] - step2;
      nxtDisplayCenteredTextLine(4, "Berr: %6d",motorBError);
      nxtDisplayCenteredTextLine(5, "Cerr: %6d",motorCError);
      wait10Msec(200);

      step1 +=160;
      step2 +=123;
     }
     nSyncedMotors = synchNone;
     nMotorEncoder[motorB] = 0;
     nMotorEncoder[motorC] = 0;
     motorBError = 0;
     motorCError = 0;
     step1 = 160;
     step2 = 123;

     while(nMotorEncoder[motorB]<6560)
     {
       //eraseDisplay();

       nMotorPIDSpeedCtrl[motorB] = mtrEncoderReg;
       nMotorPIDSpeedCtrl[motorC] = mtrSyncRegSlave;
	     nSyncedMotors = synchCB;
	     nSyncedTurnRatio = 77; //move forward
       nMotorEncoderTarget[motorC] = 160;
	     motor[motorC] =70;
       while(nMotorRunState[motorC] != runStateIdle) {}
       nxtDisplayCenteredTextLine(2, "B: %6d",nMotorEncoder[motorB]);
       nxtDisplayCenteredTextLine(3, "C: %6d",nMotorEncoder[motorC]);
       setTemp();
       checkLocalCell();
       motorBError = nMotorEncoder[motorB] - step2;
       motorCError = nMotorEncoder[motorC] - step1;
      // wait10Msec(100);
        nxtDisplayCenteredTextLine(4, "Berr: %6d",motorBError);
      nxtDisplayCenteredTextLine(5, "Cerr: %6d",motorCError);
      //wait10Msec(100);
      nSyncedMotors = synchNone;
      nMotorEncoderTarget[motorB] = motorBError;
      nMotorEncoderTarget[motorC] = motorCError;
      if(motorBError>0)
      {
        motor[motorB] = -15;
      }
      else if(motorBError<0)
      {
      	motor[motorB] = 15;
      }
      if(motorCError>0)
      {
      	motor[motorC] = -15;
      }
      else if(motorCError)
      {
        motor[motorC] = 15;
      }
      while(nMotorRunState[motorB] != runStateIdle || nMotorRunState[motorC] != runStateIdle) {}
      step1 +=180;
      step2 +=90;
       }

    //doTurn();
     nSyncedMotors = synchNone;
     nMotorEncoder[motorB] = 0;
     nMotorEncoder[motorC] = 0;
     motorBError = 0;
     motorCError = 0;
     step1 = 180;
     step2 = 90;*/

  }
    SaveNxtDatalog();

}
