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
#include "poseCell_list.h";

//----General In-Module Variables----//
int numActive = 0; //number of active cells
int currentTheta = 0; //theta for pose and turns
int currentDirection = 0; //direction for pose and turns
int changeTheta = 0;
int nextEmptyCell = 0; //used for holding the next empty cell in the localcell Struct
int listOneActive = 0;
int listTwoActive = 0;
int clicks = 180;
int totalClicks = 0;
char read = 1;//determines which list is being written to and which is being read from
///////..../////////
//read = 1 means writing to list one - reading from list two
//read = 0 means writing to list two - reading from list one
///////..../////////

/*
/ Things to do.
| 1)
| 2)
| 3) test
\ 4) see if 200 is big enough list size - could be heaps of cells active b4 inhibition kicks in
| 5) test binary search with the sorted list and test more with the list sorter - then test entire thing - then test with the visual display
|
|
\
|
|
|
\
\
*/



//                           //
//----Pose Cell Functions----//
//                           //

void changeRead()
{
  if(read)
  {
    read = 0;
  }
  else
  {
  read = 1;
  }

}

//----Clear Lists----//
void clear_One()
{
  memset(poseWorld.poseListOne,0, 7*listLength);
	listOneActive = 0;
	nextEmptyCell = 0;
}

void clear_Two()
{
	memset(poseWorld.poseListTwo,0, 7*listLength);
	listTwoActive = 0;
	nextEmptyCell = 0;
}

//----Binary search----//
int binarySearchListOne(char xSearch, char ySearch, char thetaSearch)
{
       int low = 0;
       int high = listOneActive - 1;
       while (low <= high)
       {
          int mid = (int) (low + ((high - low) / 2));
          if (poseWorld.poseListOne[mid].x > xSearch)
          {
            high = mid - 1;
          }
          else if (poseWorld.poseListOne[mid].x < xSearch)
          {
            low = mid + 1;
          }
          else
          {
	          if (poseWorld.poseListOne[mid].y > ySearch)
	          {
	            high = mid - 1;
	          }
	          else if (poseWorld.poseListOne[mid].y < ySearch)
	          {
	            low = mid + 1;
	          }
          	else
          	{
		          if (poseWorld.poseListOne[mid].theta > thetaSearch)
		          {
		            high = mid - 1;
		          }
		          else if (poseWorld.poseListOne[mid].theta < thetaSearch)
		          {
		            low = mid + 1;
		          }
		          else
		          {
		            return mid;
		          }
          	}
          }
       }
       return -1; // not found
}

int binarySearchListTwo(char xSearch, char ySearch, char thetaSearch)
{
       int low = 0;
       int high = listTwoActive - 1;
       while (low <= high)
       {
          int mid = (int) (low + ((high - low) / 2));
          if (poseWorld.poseListTwo[mid].x > xSearch)
          {
            high = mid - 1;
          }
          else if (poseWorld.poseListTwo[mid].x < xSearch)
          {
            low = mid + 1;
          }
          else
          {
	          if (poseWorld.poseListTwo[mid].y > ySearch)
	          {
	            high = mid - 1;
	          }
	          else if (poseWorld.poseListTwo[mid].y < ySearch)
	          {
	            low = mid + 1;
	          }
          	else
          	{
		          if (poseWorld.poseListTwo[mid].theta > thetaSearch)
		          {
		            high = mid - 1;
		          }
		          else if (poseWorld.poseListTwo[mid].theta < thetaSearch)
		          {
		            low = mid + 1;
		          }
		          else
		          {
		            return mid;
		          }
          	}
          }
       }
       return -1; // not found
}

//----searches list for same cell previously added----//
int searchList(char xSearch, char ySearch, char thetaSearch)
{
  if(read)
  {
    return binarySearchListTwo(xSearch, ySearch, thetaSearch);
  }
  else
  {
    return binarySearchListOne(xSearch, ySearch, thetaSearch);
  }

}
/*int searchList(char xSearch, char ySearch, char thetaSearch)
{
  int lVar;
  if(read)
  {
    for(lVar = 0; lVar < listTwoActive; lVar++)
    {
      if(poseWorld.poseListTwo[lVar].x == xSearch)
      {
        if(poseWorld.poseListTwo[lVar].y == ySearch)
        {
          if(poseWorld.poseListTwo[lVar].theta == thetaSearch)
          {
            return lVar;
          }
        }
      }
    }
  return -1;
  }
  else
  {
    for(lVar = 0; lVar < listOneActive; lVar++)
    {
      if(poseWorld.poseListOne[lVar].x == xSearch)
      {
        if(poseWorld.poseListOne[lVar].y == ySearch)
        {
          if(poseWorld.poseListOne[lVar].theta == thetaSearch)
          {
            return lVar;
          }
        }
      }
    }
  return -1;
  }
}*/


void findMax()
{
  float max = 0;
  int maxX, maxY, maxTheta, loopCount;
  if(read)
  {
    for(loopCount = 0; loopCount < listOneActive; loopCount++)
    {
      if(poseWorld.poseListOne[loopCount].cellActivation > max)
      {
        max = poseWorld.poseListOne[loopCount].cellActivation;
        maxX = poseWorld.poseListOne[loopCount].x;
        maxY = poseWorld.poseListOne[loopCount].y;
        maxTheta = poseWorld.poseListOne[loopCount].theta;
      }
    }
    poseWorld.maxActivatedCell.x = maxX;
    poseWorld.maxActivatedCell.y = maxY;
    poseWorld.maxActivatedCell.theta = maxTheta;
    poseWorld.maxCellActivation = max;
  }
  else
  {
    for(loopCount = 0; loopCount < listTwoActive; loopCount++)
    {
      if(poseWorld.poseListTwo[loopCount].cellActivation > max)
      {
        max = poseWorld.poseListTwo[loopCount].cellActivation;
        maxX = poseWorld.poseListTwo[loopCount].x;
        maxY = poseWorld.poseListTwo[loopCount].y;
        maxTheta = poseWorld.poseListTwo[loopCount].theta;
      }
    }
    poseWorld.maxActivatedCell.x = maxX;
    poseWorld.maxActivatedCell.y = maxY;
    poseWorld.maxActivatedCell.theta = maxTheta;
    poseWorld.maxCellActivation = max;
  }
}

//----adds cells to poseList----//
void addToList(char xAdd, char yAdd, char thetaAdd, float actAdd)
{
  int searchRes = searchList(xAdd, yAdd, thetaAdd);
  if(searchRes == -1)
  {
    if(read)
    {
      poseWorld.poseListTwo[nextEmptyCell].x = xAdd;
      poseWorld.poseListTwo[nextEmptyCell].y = yAdd;
      poseWorld.poseListTwo[nextEmptyCell].theta = thetaAdd;
      poseWorld.poseListTwo[nextEmptyCell].cellActivation = actAdd;
      listTwoActive++;
    }
    else
    {
      poseWorld.poseListOne[nextEmptyCell].x = xAdd;
      poseWorld.poseListOne[nextEmptyCell].y = yAdd;
      poseWorld.poseListOne[nextEmptyCell].theta = thetaAdd;
      poseWorld.poseListOne[nextEmptyCell].cellActivation = actAdd;
      listOneActive++;
    }
    nextEmptyCell++;
  }
  else
  {
    if(read)
    {
      poseWorld.poseListTwo[searchRes].cellActivation = actAdd;
    }
    else
    {
      poseWorld.poseListOne[searchRes].cellActivation = actAdd;
    }
  }
}

//----adds cells to poseList----//
void addToStartList(char xAdd, char yAdd, char thetaAdd, float actAdd)
{
      poseWorld.poseListOne[nextEmptyCell].x = xAdd;
      poseWorld.poseListOne[nextEmptyCell].y = yAdd;
      poseWorld.poseListOne[nextEmptyCell].theta = thetaAdd;
      poseWorld.poseListOne[nextEmptyCell].cellActivation = actAdd;
      listOneActive++;
}

//----adds cells only to the write list (only use in setExcitation)----//
void addToWriteList(char xAdd, char yAdd, char thetaAdd, float actAdd)
{
  int searchRes = searchList(xAdd, yAdd, thetaAdd);
  if(searchRes == -1)
  {
    if(read)
    {
      poseWorld.poseListTwo[nextEmptyCell].x = xAdd;
      poseWorld.poseListTwo[nextEmptyCell].y = yAdd;
      poseWorld.poseListTwo[nextEmptyCell].theta = thetaAdd;
      poseWorld.poseListTwo[nextEmptyCell].cellActivation = actAdd;
      listTwoActive++;
    }
    else
    {
      poseWorld.poseListOne[nextEmptyCell].x = xAdd;
      poseWorld.poseListOne[nextEmptyCell].y = yAdd;
      poseWorld.poseListOne[nextEmptyCell].theta = thetaAdd;
      poseWorld.poseListOne[nextEmptyCell].cellActivation = actAdd;
      listOneActive++;
    }
    nextEmptyCell++;
  }
  else
  {
    if(read)
    {
      poseWorld.poseListTwo[searchRes].cellActivation += actAdd;
    }
    else
    {
      poseWorld.poseListOne[searchRes].cellActivation += actAdd;
    }
  }
}

//----Sorts the lists----//
void shell_sortListOne() {
  int i, j, increment;
  poseCell temp;
  increment = (int) listOneActive / 2;

  while (increment > 0)
  {
    for (i = increment; i < listOneActive; i++) {
      j = i;
      memcpy(temp, poseWorld.poseListOne[i],8);

      while((j >= increment) && (poseWorld.poseListOne[j-increment].x > temp.x))
      {
        memcpy(poseWorld.poseListOne[j],poseWorld.poseListOne[j-increment],8);
        j = j - increment;
      }
      while((j >= increment) && (poseWorld.poseListOne[j-increment].x == temp.x))
      {
        if((poseWorld.poseListOne[j-increment].y > temp.y))
        {
          memcpy(poseWorld.poseListOne[j],poseWorld.poseListOne[j-increment],8);
          j = j - increment;
        }
        else {break;}
      }
      while((j >= increment) && (poseWorld.poseListOne[j-increment].x == temp.x))
      {
        if((poseWorld.poseListOne[j-increment].y == temp.y) && (poseWorld.poseListOne[j-increment].theta > temp.theta))
        {
          memcpy(poseWorld.poseListOne[j],poseWorld.poseListOne[j-increment],8);
          j = j - increment;
        }
        else {break;}
      }
      memcpy(poseWorld.poseListOne[j],temp,8);
    }

    if (increment == 2)
       increment = 1;
    else
       increment = (int) (increment / 2.2);
  }
}

void shell_sortListTwo() {
  int i, j, increment;
  poseCell temp;
  increment = (int) listTwoActive / 2;
  while (increment > 0)
  {
    for (i = increment; i < listTwoActive; i++) {
      j = i;
      memcpy(temp, poseWorld.poseListTwo[i],8);

      while((j >= increment) && (poseWorld.poseListTwo[j-increment].x > temp.x))
      {
        memcpy(poseWorld.poseListTwo[j],poseWorld.poseListTwo[j-increment],8);
        j = j - increment;
      }
      while((j >= increment) && (poseWorld.poseListTwo[j-increment].x == temp.x))
      {
        if((poseWorld.poseListTwo[j-increment].y > temp.y))
        {
          memcpy(poseWorld.poseListTwo[j],poseWorld.poseListTwo[j-increment],8);
          j = j - increment;
        }
        else {break;}
      }
      while((j >= increment) && (poseWorld.poseListTwo[j-increment].x == temp.x))
      {
        if((poseWorld.poseListTwo[j-increment].y == temp.y) && (poseWorld.poseListTwo[j-increment].theta > temp.theta))
        {
          memcpy(poseWorld.poseListTwo[j],poseWorld.poseListTwo[j-increment],8);
          j = j - increment;
        }
        else {break;}
      }
      memcpy(poseWorld.poseListTwo[j],temp,8);
    }

    if (increment == 2)
       increment = 1;
    else
       increment = (int) (increment / 2.2);
  }
}

void sortList()
{
	if(read)
  {
    shell_sortListTwo();
  }
  else
  {
    shell_sortListOne();
  }

}

//----initialises the start cell and sets starting activation----//
void startCell()
{
  poseWorld.maxActivatedCell.x = 5;
  poseWorld.maxActivatedCell.y = 5;
  poseWorld.maxActivatedCell.theta = 0;
  poseWorld.maxCellActivation = 0.5;
  addToStartList(5,5,0,0.5);
}

//----initalises all pose cells as inactive and sets all pose activity to zero----//
void setupPoseStructure()
{
  memset(poseWorld.poseListOne, 0, 1400);
  memset(poseWorld.poseListTwo, 0, 1400);
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
  int searchRes;

  //set values
  searchRes = searchList(cellX, cellY, cellTheta);
  if(searchRes == -1)
  {
    previousActivation = 0;
  }
  else
  {
    if(read)
    {
      previousActivation = poseWorld.poseListTwo[searchRes];
    }
    else
    {
      previousActivation = poseWorld.poseListOne[searchRes];
    }
  }

  maxActivation = poseWorld.maxCellActivation;

  if(previousActivation == activation)
  {//if the same do nothing, exit function
    return;
  }
  changeRead();
  addToList(cellX,cellY,cellTheta,activation);
   changeRead();
  if(activation > maxActivation)
  {//if activation greater than max, then set as maximum
    poseWorld.maxActivatedCell.x = cellX;
    poseWorld.maxActivatedCell.y = cellY;
    poseWorld.maxActivatedCell.theta = cellTheta;
    poseWorld.maxCellActivation = activation;
  }
}
/*
//----Injects additional activity into cells----//
void injectEnergy (float stepSize, char xCell, char yCell, char thetaCell)
{
  float previousActivation = poseWorld.poseActivity[poseAssoc.xCell].array2D[poseAssoc.yCell][poseAssoc.thetaCell];
  setActivation(poseAssoc.xCell,poseAssoc.yCell,poseAssoc.thetaCell, previousActivation + injectionStrength * stepSize);
}
*/
//----Excites neighbouring cells----//
void doExcitation(float stepSize)
{
  //initialises loop varibles
  int i;
  char relX;
  char relY;
  char relTheta;
  float excitationWeight;

  //
  if(read)
  {
  	clear_Two();
    for(i = 0; i < listOneActive; i++)
    {
      float thisActivation = poseWorld.poseListOne[i].cellActivation;
      for (relX = -influenceXY; relX <= influenceXY; relX++)
      {
        char neighbourX = getWrappedX(poseWorld.poseListOne[i].x + relX);
        for(relY = - influenceXY; relY<= influenceXY; relY++)
        {
          char neighbourY = getWrappedY(poseWorld.poseListOne[i].y + relY);
          for(relTheta = -influenceTheta; relTheta <= influenceTheta; relTheta++)
          {
            char neighbourTheta = getWrappedTheta(poseWorld.poseListOne[i].theta + relTheta);
            excitationWeight = excitation_Weights[relX + influenceXY].array2D[relY + influenceXY][relTheta + influenceTheta];
            if((thisActivation * excitationWeight * stepSize)>0)
            {
              addToWriteList(neighbourX,neighbourY,neighbourTheta, (thisActivation * excitationWeight * stepSize));
            }
          }
        }
      }
      sortList();
    }
  }
  else
  {
  	clear_One();
    for(i = 0; i < listTwoActive; i++)
    {
      float thisActivation = poseWorld.poseListTwo[i].cellActivation;
      for (relX = -influenceXY; relX <= influenceXY; relX++)
      {
        char neighbourX = getWrappedX(poseWorld.poseListOne[i].x + relX);
        for(relY = - influenceXY; relY<= influenceXY; relY++)
        {
          char neighbourY = getWrappedY(poseWorld.poseListOne[i].y + relY);
          for(relTheta = -influenceTheta; relTheta <= influenceTheta; relTheta++)
          {
            char neighbourTheta = getWrappedTheta(poseWorld.poseListOne[i].theta + relTheta);
            excitationWeight = excitation_Weights[relX + influenceXY].array2D[relY + influenceXY][relTheta + influenceTheta];
            if((thisActivation * excitationWeight * stepSize)>0)
            {
              addToWriteList(neighbourX,neighbourY,neighbourTheta, (thisActivation * excitationWeight * stepSize));
            }
          }
        }
      }
      sortList();
    }
  }

}

//----Inhibits neighbouring cells----//
float doInhibition(float stepSize)
{
  float inhibition = globalInhibition * stepSize;
  float activationSum = 0;
  float activation;
  numActive = 0; //clear number of active cells counter

  //initialise loop variable
  int i;

  if(read)
  {
    clear_One();
    changeRead();
    for(i = 0; i < listTwoActive; i++)
    {
      activation = poseWorld.poseListTwo[i].cellActivation - inhibition;
      if(activation <=0)
      {
       activation = 0; //cant have negative activity
      }
      //setActivation(poseWorld.poseListTwo[i].x, poseWorld.poseListTwo[i].y, poseWorld.poseListTwo[i].theta, activation); //set activation and see if is maximum

      //poseWorld.poseListTwo[i].cellActivation = activation;
      if(activation>0)
      {
        addToList(poseWorld.poseListTwo[i].x,poseWorld.poseListTwo[i].y,poseWorld.poseListTwo[i].theta,activation);
        numActive++; //increase number of active cells
        activationSum += activation;
      }
    }
  }
  else
  {
    clear_Two();
    changeRead();
    for(i = 0; i < listOneActive; i++)
    {
      activation = poseWorld.poseListOne[i].cellActivation - inhibition;
      if(activation <=0)
      {
       activation = 0; //cant have negative activity
      }
      //setActivation(poseWorld.poseListTwo[i].x, poseWorld.poseListTwo[i].y, poseWorld.poseListTwo[i].theta, activation); //set activation and see if is maximum

      //poseWorld.poseListTwo[i].cellActivation = activation;
      if(activation>0)
      {
        addToList(poseWorld.poseListTwo[i].x,poseWorld.poseListTwo[i].y,poseWorld.poseListTwo[i].theta,activation);
        numActive++; //increase number of active cells
        activationSum += activation;
      }
    }
  }
  changeRead();
  findMax();
  nextEmptyCell=numActive;
  return activationSum;
}

//----Normalises the activity in the pose matrix----//
void doNormalisation(float activationSum)
{//requires activationSum from inhibition

  //initalise loop variables
  int i;
  if(read)
  {
    for(i = 0; i < listOneActive; i++)
    {
      poseWorld.poseListOne[i].cellActivation /= activationSum; //normalise
    }
  }
  else
  {
    for(i = 0; i < listTwoActive; i++)
    {
      poseWorld.poseListTwo[i].cellActivation /= activationSum; //normalise
    }
  }
  findMax();
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
void pathIntegrateCell(int listCellNum, char xp, char yp, char thetap, float deltaTheta, float translation)
{
  //initialise loop variables
  char relativeX;
  char relativeY;
  char relativeTheta;

  char x;
  char y;
  char theta;

  float activation;

  if(read)
  {
    activation = poseWorld.poseListTwo[listCellNum].cellActivation;
  }
  else
  {
    activation = poseWorld.poseListOne[listCellNum].cellActivation;
  }

  float deltaPoseX = (cosDegrees(currentDirection) * translation) / 0.5;/// (lengthX / sizeX);
  float deltaPoseY = (sinDegrees(currentDirection) * translation) / 0.5; //(lengthX / sizeX);
  float deltaPoseTheta = deltaTheta / 60;//(360 / sizeTheta);

  int intOffsetX = (int) deltaPoseX; //only a whole number of cells moved
  int intOffsetY = (int) deltaPoseY;
  int intOffsetTheta = (int) deltaPoseTheta;

  getActivationDistribution(deltaPoseX - intOffsetX, deltaPoseY - intOffsetY, deltaPoseTheta - intOffsetTheta);

  changeRead();
  for(relativeX = 0; relativeX < 2; relativeX++)
  {
    x = getWrappedX(xp + intOffsetX + relativeX);
    for(relativeY = 0; relativeY < 2; relativeY++)
    {
      y = getWrappedY(yp + intOffsetY + relativeY);
      for(relativeTheta = 0; relativeTheta < 2; relativeTheta++)
      {
        theta = getWrappedTheta(thetap + intOffsetTheta + relativeTheta);
        if((distribution[relativeX].array2D[relativeY][relativeTheta] * activation) > 0)
        {
          addToWriteList(x,y,theta, (distribution[relativeX].array2D[relativeY][relativeTheta] * activation));
        }
      }
    }
  }
  changeRead();
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
  //sortList();
  activeSum = doInhibition(stepSize);
  doNormalisation(activeSum);
}

//----handles pose thing----//
void pose3D(float deltaTheta, float translation)
{
  //initialise loop variables
  char f;
  changeRead();
  if(read)
  {
    clear_One();
    for(f = 0; f < listTwoActive; f++)
    {
      pathIntegrateCell(f,poseWorld.poseListTwo[f].x, poseWorld.poseListTwo[f].y, poseWorld.poseListTwo[f].theta, deltaTheta, translation);
      shell_sortListOne();
    }

  }
  else
  {
    clear_Two();
    for(f = 0; f < listOneActive; f++)
    {
      pathIntegrateCell(f,poseWorld.poseListOne[f].x, poseWorld.poseListOne[f].y, poseWorld.poseListOne[f].theta, deltaTheta, translation);
      shell_sortListTwo();
    }

  }

  //changeRead();
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
