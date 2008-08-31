#pragma config(Sensor, S1,     leftSonar,           sensorSONAR)
#pragma config(Sensor, S2,     centreSonar,         sensorSONAR)
#pragma config(Sensor, S3,     rightSonar,          sensorSONAR)
#pragma config(Motor,  motorB,          leftMotor,            tmotorNxtEncoderClosedLoop)
#pragma config(Motor,  motorC,          rightMotor,           tmotorNxtEncoderClosedLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/////////////////////////////////////////////////////////////////////////////////////////
//
//      Title:- Pose Cells test
//      Author:- Lachlan Smith (RobotC)
//               Mark Wakabayashi (Jave version)
//               Michael Milford (original algorithim/c code)
//      Purpose:- Testing pose cells
//
//
/////////////////////////////////////////////////////////////////////////////////////////

//----include files----//
#include "poseCell.h";

//----General In-Module Variables----//
int numActive = 0; //number of active cells
int currentTheta = 0; //theta for pose and turns
int currentDirection = 0; //direction for pose and turns
int changeTheta = 0;
char nextEmptyCell = 0; //used for holding the next empty cell in the localcell Struct
int clicks = 180;
int totalClicks = 0;
int numberOfCells = 2400;

//                           //
//----Pose Cell Functions----//
//                           //

//----initialises the start cell and sets starting activation----//
void startCell()
{
  poseWorld.maxActivatedCell.x = 5;
  poseWorld.maxActivatedCell.y = 5;
  poseWorld.maxActivatedCell.theta = 0;
  poseWorld.poseActivity[5].array2D[5][0] = startActivation;
}

//----initalises all pose cells as inactive and sets all pose activity to zero----//
void setupPoseStructure()
{
  memset(tempPose, 0, 2400);
  memset(poseWorld,0,2403);
}

//----Excitation matrix----//
void excitationMatrixSetup()
{
	//done in matlab - probably should check if right
  excitation_Weights[0].array2D[0][0] =  0.0495;
  excitation_Weights[0].array2D[0][1] =  0.3655;
  excitation_Weights[0].array2D[0][2] =  0.0495;
  excitation_Weights[0].array2D[1][0] =  0.1727;
  excitation_Weights[0].array2D[1][1] =  1.2757;
  excitation_Weights[0].array2D[1][2] =  0.1727;
  excitation_Weights[0].array2D[2][0] =  0.0495;
  excitation_Weights[0].array2D[2][1] =  0.3655;
  excitation_Weights[0].array2D[2][2] =  0.0495;

  excitation_Weights[1].array2D[0][0] =  0.1727;
  excitation_Weights[1].array2D[0][1] =  1.2757;
  excitation_Weights[1].array2D[0][2] =  0.1727;
  excitation_Weights[1].array2D[1][0] =  0.6026;
  excitation_Weights[1].array2D[1][1] =  4.4528;
  excitation_Weights[1].array2D[1][2] =  0.6026;
  excitation_Weights[1].array2D[2][0] =  0.1727;
  excitation_Weights[1].array2D[2][1] =  1.2757;
  excitation_Weights[1].array2D[2][2] =  0.1727;

  excitation_Weights[2].array2D[0][0] =  0.0495;
  excitation_Weights[2].array2D[0][1] =  0.3655;
  excitation_Weights[2].array2D[0][2] =  0.0495;
  excitation_Weights[2].array2D[1][0] =  0.1727;
  excitation_Weights[2].array2D[1][1] =  1.2757;
  excitation_Weights[2].array2D[1][2] =  0.1727;
  excitation_Weights[2].array2D[2][0] =  0.0495;
  excitation_Weights[2].array2D[2][1] =  0.3655;
  excitation_Weights[2].array2D[2][2] =  0.0495;
}

//----Clears Encoders - used to ensure correct rotation angles----//
void clearEncoders()
{
	//have to unsynch motors as when clearing the master the slave wont clear and attempting to clear
	//the slave causes an error
	nSyncedMotors = synchNone;
  nMotorEncoder[motorC] = 0;
  nMotorEncoder[motorB] = 0;
}

//----Determine rotation of robot from encoder values----//
int getRotation()
{
	float motorBCount = nMotorEncoder[motorB];
	float motorCCount = nMotorEncoder[motorC];
	motorBCount /= 190; //distance between wheels in encoder clicks
	motorCCount /= 190;
	int thetaOne;
	int thetaTwo;
	int thetaThree;
	if(motorBCount<0)
	{
	  motorBCount *= -1;
	  thetaOne = radiansToDegrees(atan(motorBCount));
	  thetaTwo = radiansToDegrees(atan(motorCCount));
    return -(thetaOne + thetaTwo);
	}
	else if(motorCCount<0)
	{
    motorCCount *= -1;
	  thetaOne = radiansToDegrees(atan(motorBCount));
	  thetaTwo = radiansToDegrees(atan(motorCCount));
	  return (thetaOne + thetaTwo);
  }
  else if(motorCCount < 0 && motorBCount < 0)
  {
  	motorBCount *= -1;
  	motorCCount *= -1;
  	thetaOne = radiansToDegrees(atan(motorBCount));
	  thetaTwo = radiansToDegrees(atan(motorCCount));
	  return (thetaOne + thetaTwo);
  }
  else
  {
  	thetaOne = radiansToDegrees(atan(motorBCount));
	  thetaTwo = radiansToDegrees(atan(motorCCount));
	  return (thetaOne + thetaTwo);
  }
  return thetaThree;
}


//----Wrapping values in x-direction in Pose structure----//
char getWrappedX(char indexX)
{
  if(indexX < 0)
  {
    return (indexX + sizeX);
  }
  else if(indexX >= sizeX)
  {
    return (indexX - sizeX);
  }
  return indexX;
}

//----Wrapping values in y-direction in Pose structure----//
char getWrappedY(char indexY)
{
  if(indexY < 0)
  {
    return (indexY + sizeY);
  }
  else if(indexY >= sizeY)
  {
    return (indexY - sizeY);
  }
  return indexY;
}

//----Wrapping values in theta-direction in Pose structure----//
char getWrappedTheta(char indexTheta)
{
  if(indexTheta < 0)
  {
    return (indexTheta + sizeTheta);
  }
  else if(indexTheta >= sizeTheta)
  {
    return (indexTheta - sizeTheta);
  }
  return indexTheta;
}

//----Sets activation in cell and handles maximum cell----//
void setActivation(char cellX, char cellY, char cellTheta, float activation)
{
	float previousActivation; //previous activation of a cell
	float maxActivation; //activation of max axtivated cell

	//set values
  previousActivation = poseWorld.poseActivity[cellX].array2D[cellY][cellTheta];

  maxActivation = poseWorld.poseActivity[maxX].array2D[maxY][maxTheta];

  if(previousActivation == activation)
  {//if the same do nothing, exit function
    return;
  }
  poseWorld.poseActivity[cellX].array2D[cellY][cellTheta] = activation; //else set activation

  if(activation > maxActivation)
  {//if activation greater than max, then set as maximum
    poseWorld.maxActivatedCell.x = cellX;
    poseWorld.maxActivatedCell.y = cellY;
    poseWorld.maxActivatedCell.theta = cellTheta;
  }
}

//----Injects additional activity into cells----//
void injectEnergy (float stepSize, char xCell, char yCell, char thetaCell)
{
  float previousActivation = poseWorld.poseActivity[poseAssoc.xCell].array2D[poseAssoc.yCell][poseAssoc.thetaCell];
  setActivation(poseAssoc.xCell,poseAssoc.yCell,poseAssoc.thetaCell, previousActivation + injectionStrength * stepSize);
}

//----Excites neighbouring cells----//
void doExcitation(float stepSize)
{
  //initialises loop varibles
  char i;
  char j;
  char k;

  char relX;
  char relY;
  char relTheta;

  memcpy(tempPose, poseWorld.poseActivity, numberOfCells); //fillTempPose();

  for(i = 0; i < sizeX; i++)
  {
    for(j = 0; j < sizeY; j++)
    {
      for(k = 0; k < sizeTheta; k++)
      {
        if(poseWorld.poseActivity[i].array2D[j][k] > 0) //if cell active
        {
        	float thisActivation = poseWorld.poseActivity[i].array2D[j][k];
        	for (relX = -influenceXY; relX <= influenceXY; relX++)
        	{
            char neighbourX = getWrappedX(i + relX);
            for(relY = - influenceXY; relY<= influenceXY; relY++)
            {
              char neighbourY = getWrappedY(j + relY);
              for(relTheta = -influenceTheta; relTheta <= influenceTheta; relTheta++)
              {
                char neighbourTheta = getWrappedTheta(k + relTheta);
              	float excitationWeight = excitation_Weights[relX + influenceXY].array2D[relY + influenceXY][relTheta + influenceTheta];
                tempPose[neighbourX].array2D[neighbourY][neighbourTheta] += (thisActivation * excitationWeight * stepSize);
              }
            }
        	}
        }
      }
    }
  }
  memcpy(poseWorld.poseActivity, tempPose, numberOfCells); //fillFinalPose();
}

//----Inhibits neighbouring cells----//
float doInhibition(float stepSize)
{
	float inhibition = globalInhibition * stepSize;
  float activationSum = 0;
  numActive = 0; //clear number of active cells counter

  //initialise loop variable
  char i;
  char j;
  char k;

  for(i = 0; i < sizeX; i++)
  {
    for(j = 0; j < sizeY; j++)
    {
      for(k = 0; k < sizeTheta; k++)
      {
        if(poseWorld.poseActivity[i].array2D[j][k] > 0) //if cell active
        {
        	float activation = poseWorld.poseActivity[i].array2D[j][k] - inhibition;
          if(activation <=0)
          {
            activation = 0; //cant have negative activity
          }
          setActivation(i, j, k, activation); //set activation and see if is maximum
          activationSum += activation;
          if(activation>0)
          {
            numActive++; //increase number of active cells
          }
        }
      }
    }
  }
  return activationSum;
}

//----Normalises the activity in the pose matrix----//
void doNormalisation(float activationSum)
{//requires activationSum from inhibition

  //initalise loop variables
	char i;
  char j;
  char k;

  for(i = 0; i < sizeX; i++)
  {
    for(j = 0; j < sizeY; j++)
    {
      for(k = 0; k < sizeTheta; k++)
      {
        if(poseWorld.poseActivity[i].array2D[j][k] > 0) //if active
        {
        	poseWorld.poseActivity[i].array2D[j][k] = poseWorld.poseActivity[i].array2D[j][k] / activationSum; //normalise
        }
      }
    }
  }
}

//----Sets up the distribution for activity----//
void getActivationDistribution(float offsetX, float offsetY, float offsetTheta)
{
  //initialise variables
  char signX;
  char signY;
  char signTheta;
  float portion;

  char i;
  char j;
  char k;

  signX = -1;
  for(i = 0; i < 2; i++)
  {
    signY = -1;
    for(j = 0; j < 2; j++)
    {
      signTheta = -1;
      for(k = 0; k < 2; k++)
      {
        portion =  ((1-i) + signX * offsetX) * ((1-j) + signY * offsetY) * ((1-k) + signTheta * offsetTheta);
        distribution[i].array2D[j][k] = portion;
        signTheta = +1;
      }
      signY = +1;
    }
    signX = +1;
  }
}

//----Shifts the activity in the pose structure----//
void pathIntegrateCell(char xp, char yp, char thetap, float deltaTheta, float translation)
{
  //initialise loop variables
	char relativeX;
	char relativeY;
	char relativeTheta;

	char x;
	char y;
	char theta;

	float deltaPoseX = (cosDegrees(currentDirection) * translation) / 0.5;/// (lengthX / sizeX);
  float deltaPoseY = (sinDegrees(currentDirection) * translation) / 0.5; //(lengthX / sizeX);
	float deltaPoseTheta = deltaTheta / 60;//(360 / sizeTheta);

  int intOffsetX = (int) deltaPoseX; //only a whole number of cells moved
  int intOffsetY = (int) deltaPoseY;
  int intOffsetTheta = (int) deltaPoseTheta;

  getActivationDistribution(deltaPoseX - intOffsetX, deltaPoseY - intOffsetY, deltaPoseTheta - intOffsetTheta);

  for(relativeX = 0; relativeX < 2; relativeX++)
  {
  	x = getWrappedX(xp + intOffsetX + relativeX);
    for(relativeY = 0; relativeY < 2; relativeY++)
    {
      y = getWrappedY(yp + intOffsetY + relativeY);
    	for(relativeTheta = 0; relativeTheta < 2; relativeTheta++)
    	{
    	  theta = getWrappedTheta(thetap + intOffsetTheta + relativeTheta);
    	  tempPose[x].array2D[y][theta] += distribution[relativeX].array2D[relativeY][relativeTheta] * poseWorld.poseActivity[xp].array2D[yp][thetap];
    	}
    }
  }
}

//----Sets up Pose Cells----//
void initialisePose()
{
	setupPoseStructure();
  excitationMatrixSetup();
  startCell();
  numActive = 1;
}

//----handles all cell stuff----//
void iterate(float stepSize)
{
	float activeSum;
  doExcitation(stepSize);
  activeSum = doInhibition(stepSize);
  doNormalisation(activeSum);
}

//----handles pose thing----//
void pose3D(float deltaTheta, float translation)
{
  //initialise loop variables
  char f;
  char g;
  char h;
  memset(tempPose, 0, 2400); //a zero matrix to perform path integration on
  for(f = 0; f < sizeX; f++)
  {
    for(g = 0; g < sizeY; g++)
    {
      for(h = 0; h < sizeTheta; h++)
      {
        if(poseWorld.poseActivity[f].array2D[g][h] > 0) //if active
        {
          pathIntegrateCell(f, g, h, deltaTheta, translation);
        }
      }
    }
  }
  memcpy(poseWorld.poseActivity,tempPose,numberOfCells);
  iterate(stepSize);
}

//----Display Max Activated cell----//
void displayMax()
{
	char tempX, tempY, tempTheta;
  tempX = (char) poseWorld.maxActivatedCell.x;
  tempY = (char) poseWorld.maxActivatedCell.y;
  tempTheta = (char) poseWorld.maxActivatedCell.theta;
	eraseDisplay();
  nxtDisplayTextLine(1, "pose: %2d,%2d", tempX, tempY);
  nxtDisplayStringAt(68, 55, ",%2d", tempTheta);
}

//----Logs data - for testing only----//
void datalogging()
{
	char tempX, tempY, tempTheta;
  tempX = (char) poseWorld.maxActivatedCell.x;
  tempY = (char) poseWorld.maxActivatedCell.y;
  tempTheta = (char) poseWorld.maxActivatedCell.theta;
	AddToDatalog(1,tempX);
  AddToDatalog(2,tempY);
  AddToDatalog(3,tempTheta);
  AddToDatalog(4,numActive);
  AddToDatalog(5,'0'); //just a separator
}

//----test drives (lol)----//
void drive(char synchRatio, int travelDistance, char speed)
{
	//note a 90 degree turn is ~190 encoders clicks
	clearEncoders();
	nSyncedMotors = synchBC;
	nSyncedTurnRatio = synchRatio;
	nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
	//nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
  nMotorEncoderTarget[motorB] = travelDistance;
	motor[motorB] = speed;
  while(nMotorRunState[motorB] != runStateIdle) {}
  changeTheta = getRotation();
  currentDirection += changeTheta;
  if(currentDirection > 360)
  {
  	currentDirection -= 360;
  }
}

//----main----//
task main ()
{
	nxtDisplayCenteredTextLine(3, "Pose Test");
	wait1Msec(500);
	initialisePose(); //set up
	iterate(stepSize); //run excitation etc
	currentDirection = 0; //set initial
  currentTheta = 0;
  changeTheta = 0;

  //display data
 displayMax();
    nxtDisplayTextLine(2, "Num Act.: %3d",numActive);
    nxtDisplayTextLine(4, "Direction: %1d", currentDirection);
    nxtDisplayTextLine(5, "currentTheta:%3d", currentTheta);
    nxtDisplayTextLine(6, "changeTheta:%3d", changeTheta);
	datalogging();
  while(totalClicks<1800)
	{
		alive(); //stop NXT from sleeping
    totalClicks += clicks;
    //drive(-100,190,50);
    drive(50,180,50);

    pose3D(changeTheta,0.5);

    displayMax();
    nxtDisplayTextLine(2, "Num Act.: %3d",numActive);
    nxtDisplayTextLine(4, "Direction: %1d", currentDirection);
    nxtDisplayTextLine(5, "currentTheta:%3d", currentTheta);
    nxtDisplayTextLine(6, "changeTheta:%3d", changeTheta);
    clearEncoders(); //clear encoder count
    changeTheta=0;
    datalogging();

  }
  PlaySound(soundFastUpwardTones);
  while(bSoundActive) {}
  SaveNxtDatalog();
}
