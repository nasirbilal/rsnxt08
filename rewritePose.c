/////BIG POSE CELL REWRITE !!!!

///not using any header files straight code in one file - this will be as messy as all fuck

/*
| improvements so far have been a significant reduction in memory and computation load due to not having an 3d
| array of PoseCellPositions but rather a 3d array of char which determine if active or not.  The poseActivity matrix
| is still used - this improvement occured as only using int (or char) movements so the [x], [y] and [theta] will be taken
| from the matrix numbering system itself
*/

//variables used for pose cells
const char sizeX = 10; //number of cells in X dimension
const char sizeY = 10; //number of cells in Y dimension
const char sizeTheta = 6; //number of cells in theta dimension
const char lengthX = 5; //length (x) represented by pose
const char lengthY = 5; //length (y) represented by pose
//note theta thought to represent 360 degrees
float startActivation = 0.5; //the starting activation of the first cell
char influenceXY = 1; // the level of influence that cells have on neighbouring cells
char influenceTheta = 1;
float weightVarianceXY = 0.8;//for gaussian distribution stuff
float weightVarianceTheta = 0.5;//for gaussian distribution stuff
char weightScaleFactor = 14; //strength of influence between cells
float stepSize = 0.2; //this is to account for something
float globalInhibition = 0.014;
char poseEstimationRadius = 6;
int numActive = 0;

//structures
typedef struct {
	char x;
	char y;
	char theta;
} PoseCellPosition;

typedef struct {
	float array2D[sizeY][sizeTheta]; //using an array of structures to create [][][]
} matrixPoseActivity;

typedef struct {
	float array2D[3][3];
} matrixExcite;

typedef struct {
	float array2D[2][2];
} matrixDistribution;

typedef struct {
	char array2D[sizeY][sizeTheta]; //using an array of structures to create [][][] //this will be one if active, zero if not
} matrixPositionReferences;

typedef struct {
	matrixPositionReferences positionReferences[sizeX];
	matrixPoseActivity poseActivity[sizeX];
	PoseCellPosition maxActivatedCell;
} PoseCellStructure;

//initalise structures
PoseCellStructure poseWorld;
matrixPoseActivity tempPoseActivity[sizeX]; //this is used for calculations on the activity of poses
matrixExcite excitation_Weights[3];
matrixDistribution distribution[2];
PoseCellPosition position;

/*
'
'start of functions
'
*/

//initialises the start cell and sets starting activation
void startCell()
{
  poseWorld.positionReferences[5].array2D[5][0] = 1;
  poseWorld.maxActivatedCell.x = 5;
  poseWorld.maxActivatedCell.y = 5;
  poseWorld.maxActivatedCell.theta = 0;
  poseWorld.poseActivity[5].array2D[5][0] = startActivation;
}

//initalises all pose cells as inactive and sets all pose activity to zero
void setupPoseStructure()
{
  char i;
  char j;
  char k;

  for(i = 0; i < sizeX; i++)
  {
    for(j = 0; j < sizeY; j++)
    {
      for(k = 0; k < sizeTheta; k++)
      {
        poseWorld.positionReferences[i].array2D[j][k] = 0;
        poseWorld.poseActivity[i].array2D[j][k] = 0;
        tempPoseActivity[i].array2D[j][k] = 0;
      }
    }
  }
}

//due to robotC inability to let one struct variable = another of the same sort we do it the long way
void fillTempPose()
{
	char i;
  char j;
  char k;
	for(i = 0; i < sizeX; i++)
  {
    for(j = 0; j < sizeY; j++)
    {
      for(k = 0; k < sizeTheta; k++)
      {
        tempPoseActivity[i].array2D[j][k] = poseWorld.poseActivity[i].array2D[j][k];
      }
    }
  }
}

void fillFinalPose()
{
	char i;
  char j;
  char k;
	for(i = 0; i < sizeX; i++)
  {
    for(j = 0; j < sizeY; j++)
    {
      for(k = 0; k < sizeTheta; k++)
      {
        poseWorld.poseActivity[i].array2D[j][k] = tempPoseActivity[i].array2D[j][k];
      }
    }
  }
}

void excitationMatrixSetup()
{
  excitation_Weights[0].array2D[0][0] =  0.0846;
  excitation_Weights[0].array2D[0][1] =  0.2953;
  excitation_Weights[0].array2D[0][2] =  0.0846;
  excitation_Weights[0].array2D[1][0] =  0.2953;
  excitation_Weights[0].array2D[1][1] =  1.0305;
  excitation_Weights[0].array2D[1][2] =  0.2953;
  excitation_Weights[0].array2D[2][0] =  0.0846;
  excitation_Weights[0].array2D[2][1] =  0.2953;
  excitation_Weights[0].array2D[2][2] =  0.0846;

  excitation_Weights[1].array2D[0][0] =  0.2953;
  excitation_Weights[1].array2D[0][1] =  1.0305;
  excitation_Weights[1].array2D[0][2] =  0.2953;
  excitation_Weights[1].array2D[1][0] =  1.0305;
  excitation_Weights[1].array2D[1][1] =  3.5969;
  excitation_Weights[1].array2D[1][2] =  1.0305;
  excitation_Weights[1].array2D[2][0] =  0.2953;
  excitation_Weights[1].array2D[2][1] =  1.0305;
  excitation_Weights[1].array2D[2][2] =  0.2953;

  excitation_Weights[2].array2D[0][0] =  0.0846;
  excitation_Weights[2].array2D[0][1] =  0.2953;
  excitation_Weights[2].array2D[0][2] =  0.0846;
  excitation_Weights[2].array2D[1][0] =  0.2953;
  excitation_Weights[2].array2D[1][1] =  1.0305;
  excitation_Weights[2].array2D[1][2] =  0.2953;
  excitation_Weights[2].array2D[2][0] =  0.0846;
  excitation_Weights[2].array2D[2][1] =  0.2953;
  excitation_Weights[2].array2D[2][2] =  0.0846;
}

/////////////////////////////
//                         //
//    Matrix Wrapping      //
//                         //
/////////////////////////////

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


//sets the activity in the structure and also sets maximum cell
void setActivation(char cellX, char cellY, char cellTheta, float activation)
{
	float previousActivation;
	float maxActivation;
	char maxX;
	char maxY;
	char maxTheta;

  previousActivation = poseWorld.poseActivity[cellX].array2D[cellY][cellTheta];
  maxX = poseWorld.maxActivatedCell.x;
  maxY = poseWorld.maxActivatedCell.y;
  maxTheta = poseWorld.maxActivatedCell.theta;
  maxActivation = poseWorld.poseActivity[maxX].array2D[maxY][maxTheta];

  if(previousActivation == activation)
  {
    return;
  }
  poseWorld.poseActivity[cellX].array2D[cellY][cellTheta] = activation;

  if(activation > maxActivation)
  {
    poseWorld.maxActivatedCell.x = cellX;
    poseWorld.maxActivatedCell.y = cellY;
    poseWorld.maxActivatedCell.theta = cellTheta;
  }
}

//this excites the maxActivated cell and surrounding cells
void doExcitation(float stepSize)
{
  fillTempPose();

  //for loop
  char i;
  char j;
  char k;

  char relX;
  char relY;
  char relTheta;

  for(i = 0; i < sizeX; i++)
  {
    for(j = 0; j < sizeY; j++)
    {
      for(k = 0; k < sizeTheta; k++)
      {
        if(poseWorld.positionReferences[i].array2D[j][k] == 1)
        {
        	float thisActivation = poseWorld.poseActivity[i].array2D[j][k];

        	//excite neighbouring cells
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
                tempPoseActivity[neighbourX].array2D[neighbourY][neighbourTheta] += thisActivation * excitationWeight * stepSize;
                poseWorld.positionReferences[neighbourX].array2D[neighbourY][neighbourTheta] = 1;
              }
            }
        	}
        }
      }
    }
  }
  fillFinalPose();
}

//inhibits surrounding cells
float doInhibition(float stepSize)
{
	float inhibition = globalInhibition * stepSize;
  float activationSum = 0;
  numActive = 0;

  char i;
  char j;
  char k;

  for(i = 0; i < sizeX; i++)
  {
    for(j = 0; j < sizeY; j++)
    {
      for(k = 0; k < sizeTheta; k++)
      {
        if(poseWorld.positionReferences[i].array2D[j][k] == 1)
        {
        	float activation = poseWorld.poseActivity[i].array2D[j][k] - inhibition;
          if(activation <=0)
          {
            activation = 0;
            poseWorld.positionReferences[i].array2D[j][k] = 0;
          }
          setActivation(i, j, k, activation);
          activationSum += activation;
          if(activation>0)
          {
            numActive++;
          }
        }
      }
    }
  }
  return activationSum;
}

//normalises the activity in pose structure after excitation and inhibition
void doNormalisation(float activationSum)
{

	char i;
  char j;
  char k;

  for(i = 0; i < sizeX; i++)
  {
    for(j = 0; j < sizeY; j++)
    {
      for(k = 0; k < sizeTheta; k++)
      {
        if(poseWorld.positionReferences[i].array2D[j][k] == 1)
        {
        	poseWorld.poseActivity[i].array2D[j][k] = poseWorld.poseActivity[i].array2D[j][k] / activationSum;
        }
      }
    }
  }
}

void getActivationDistribution(float offsetX, float offsetY, float offsetTheta)
{
  //distribution
  char signX;
  char signY;
  char signTheta;
  float portion;
  signX = -1;
  //loop stuff
  char i;
  char j;
  char k;

  for(i = 0; i < 2; i++)
  {
    signY = -1;
    for(j = 0; j < 2; j++)
    {
      signTheta = -1;
      for(k = 0; k < 2; k++)
      {
        portion =   ((1-i) + signX * offsetX) * ((1-j) + signY * offsetY) * ((1-k) + signTheta * offsetTheta);
        distribution[i].array2D[j][k] = portion;
        signTheta = +1;
      }
      signY = +1;
    }
    signX = +1;
  }
}

//this function handles path integration

void pathIntegrateCell(char xp, char yp, char thetap, float deltaTheta, float translation)
{
  //loop stuff
	char relativeX;
	char relativeY;
	char relativeTheta;

	char x;
	char y;
	char theta;

	float deltaPoseX = (cosDegrees(deltaTheta) * translation) / 0.5;/// (lengthX / sizeX);
  float deltaPoseY = (sinDegrees(deltaTheta) * translation) / 0.5; //(lengthX / sizeX);
	float deltaPoseTheta = deltaTheta / 60;//(360 / sizeTheta);

  int intOffsetX = (int) deltaPoseX;
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
    	  poseWorld.poseActivity[x].array2D[y][theta] += distribution[relativeX].array2D[relativeY][relativeTheta] * poseWorld.poseActivity[xp].array2D[yp][thetap];
    	}
    }
  }
}

void initialisePose()
{
	setupPoseStructure();
  excitationMatrixSetup();
  startCell();

  numActive = 1;
}

//////////////////////////////////////
//                                  //
//   Function that runs everthing   //
//                                  //
//////////////////////////////////////

void pose3D(float deltaTheta, float translation)
{
  //loop
  char i;
  char j;
  char k;

  //variables
  float activeSum;

  //correct any cells that are active and shouldn't be
  for(i = 0; i < 10; i++)
  {
    for(j = 0; j < 10; j++)
    {
      for(k = 0; k < 6; k++)
      {
        if(poseWorld.positionReferences[i].array2D[j][k] == 1)
        {

          pathIntegrateCell(i, j, k, deltaTheta, translation)
        }
      }
    }
  }
  doExcitation(stepSize);
  activeSum = doInhibition(stepSize);
  doNormalisation(activeSum);
}

task main()
{
	initialisePose();
	doExcitation(stepSize);
  float activeSum1 = doInhibition(stepSize);
  doNormalisation(activeSum1);
	while(1)
	{
		eraseDisplay();
		char top =  poseWorld.maxActivatedCell.y;
		char left = poseWorld.maxActivatedCell.x;
		char angle = poseWorld.maxActivatedCell.theta;
		float maxActivation = poseWorld.poseActivity[left].array2D[top][angle];
		nxtDisplayStringAt(0, 60, "%d", top);
		nxtDisplayStringAt(8, 60, "%d", left);
		nxtDisplayStringAt(16, 60, "%d", angle);
		//nxtDisplayString(1, "%d, %d", top, left);
		nxtDisplayString(2, "Act = %4.3f", maxActivation);
		nxtDisplayString(3, "Num Act. = %d", numActive);
	  wait10Msec(200);
	  //pose3D(10,10);
  }
}
/*
`problems - so far it is not exciting any surrounding cells - need to fix this
*/
