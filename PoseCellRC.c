/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Title:- Pose Cells
//      Author:- Lachlan Smith (RobotC)
//                                       Mark Wakabayashi (Jave version)
//                                       Michael Milford (original algorithim/c code)
//      Purpose:- The purpose of this program is to implement pose cells and path integration in RobotC for the Lego NXT.  This version will ignore
//                                        local view calibration and attempt to display the active pose cell on the NXT display using built in drawing functions in a purely
//                                        2d form, only showing a x and y plot and ignoring theta. // // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Changes made from orignal code to make this work.
//                              - cell sizes are now smaller, going to try a 10x10x6 instead
//                              - only going to use int instead of doubles, or even chars.
//                              - establish excitation matrix by hand (used matlab)
//
//
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#pragma platform(NXT)
///////////////////////////
//                       //
// Included Files        //
//                       //
///////////////////////////

#include "PoseCellRC.h"

///////////////////////////
//                       //
// Global Variables      //
//                       //
///////////////////////////

// in header file

/////////////////////////////
//                         //
// Excitatory Matrix stuff //
//                         //
/////////////////////////////

//established it like this to try and take some computational strain off NXT


//initalise all pose cell attributes
void setupPoseStructure()
{
	for(char i = 0; i < 10; i++)
	{
		for(char j = 0; j < 10; j++)
		{
			for(char k = 0; k < 6; k++)
			{
				poseEnvironment.positionReferences[i].array2D[j][k].x = i;
				poseEnvironment.positionReferences[i].array2D[j][k].y = j;
				poseEnvironment.positionReferences[i].array2D[j][k].theta = k;
				poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE = 0;
			}
		}
	}
}

//wrapping stuff
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

//to ensure that cells are active only if have some activity
void checkActive()
{
	for(char i = 0; i < 10; i++)
	{
		for(char j = 0; j < 10; j++)
		{
			for(char k = 0; k < 6; k++)
			{
				if(poseEnvironment.poseActivity[i].array2D[j][k] > 0)
				{
					poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE = 1;
				}
			}
		}
	}
}
/*
//using setActivation
void setActivition(char x, char y, char theta, char ACTIVE, double activation)
{

	PoseCellPosition cell;
	cell.x = x;
	cell.y = y;
	cell.theta = theta;
	cell.ACTIVE = ACTIVE;

	double previousActivation = poseEnvironment.poseActivity[x].array2D[y][theta];

	PoseCellPosition maxPose;
	maxPose.x = poseEnvironment.maxActivatedCell.x;
	maxPose.y = poseEnvironment.maxActivatedCell.y;
	maxPose.theta = poseEnvironment.maxActivatedCell.theta;

	if(previousActivation == activation)
	{
		return;
	}
	if(activation == 0)
	{
		ACTIVE = 0;
	}
	else
	{
		poseEnvironment.positionReferences[x].array2D[y][theta].ACTIVE = 1;
	}
	poseEnvironment.poseActivity[x].array2D[y][theta] = activation;
	if(activation > PoseCellStructure.poseActivity[maxPose.x].array2D[maxPose.y][maxPose.theta])
	{
		poseEnvironment.maxActivatedCell.x = x;
		poseEnvironment.maxActivatedCell.y = y;
		poseEnvironment.maxActivatedCell.theta = theta;
	}
}
*/

//inhibition stuff
double doInhibition(double stepSize)
{
	float inhibition = 0.0028;

	double activationSum = 0;

	for(char i = 0; i < 10; i++)
	{
		for(char j = 0; j < 10; j++)
		{
			for(char k = 0; k < 6; k++)
			{
				PoseCellPosition position1;
				position1.x = poseEnvironment.positionReferences[i].array2D[j][k].x;
				position1.y = poseEnvironment.positionReferences[i].array2D[j][k].y;
				position1.theta = poseEnvironment.positionReferences[i].array2D[j][k].theta;
				position1.ACTIVE = poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE;
				if(position1.ACTIVE) //only touch cells that are active - not the best way of doing this but will do for testing
				{
					double activation = poseEnvironment.poseActivity[i].array2D[j][k] - inhibition;
					if(activation <= 0)
					{
					activation = 0;
					}
					//setActivition(position1.x, position1.y, position1.theta, position1.ACTIVE, activation);
					activationSum += activation;
				}
			}
		}
	}
	return activationSum;
}

//Normalisation of activity
void doNormalisation(double activationSum)
{
	for(char x = 0; x < 10; x++)
	{
		for(char y = 0; y < 10; y++)
		{
			for(char z = 0; z < 6; z++)
			{
				char isActive = poseEnvironment.positionReferences[x].array2D[y][z].ACTIVE;
				if(isActive)
				{
					poseEnvironment.poseActivity[x].array2D[y][z] /= activationSum;
				}
			}
		}
	}
}

//Excitation
void doExcitation(float stepsize)
{
	for(char i = 0; i < 10; i++)
	{
		for(char j = 0; j < 10; j++)
		{
			for(char k = 0; k < 6; k++)
			{
				PoseCellPosition position2;
				position2.x = poseEnvironment.positionReferences[i].array2D[j][k].x;
				position2.y = poseEnvironment.positionReferences[i].array2D[j][k].y;
				position2.theta = poseEnvironment.positionReferences[i].array2D[j][k].theta;
				position2.ACTIVE = poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE;
				if(position2.ACTIVE)
				{
					double thisActivation = poseEnvironment.poseActivity[i].array2D[j][k];
					for(char relX = -influenceXY; relX <= influenceXY; relX++)
					{
						char neighbourX = getWrappedX(position2.x + relX);
						for(char relY = -influenceXY; relY <= influenceXY; relY++)
						{
							char neighbourY = getWrappedY(position2.y + relY);
							for(char relTheta = -influenceTheta; relTheta <= influenceTheta; relTheta++)
							{
								char neighbourTheta = getWrappedTheta(position2.theta + relTheta);
								float excitationWeight = (float) excitation_Weights[relX + influenceXY].array2D[relY + influenceXY][relTheta + influenceTheta] * 0.2;
								poseEnvironment.poseActivity[neighbourX].array2D[neighbourY][neighbourTheta] += thisActivation * excitationWeight;
							}
						}
					}
				}
			}
		}
	}
}

///////////////////////////
//                       //
// Pose Cell start       //
//                       //
///////////////////////////


//Determine Startcell - currently using a global starting Pose
void initalisePose()
{
	excitationMatrixSetup();
	setupPoseStructure();
	PoseCellPosition startPosition;
	startPosition.x = 5;
	startPosition.y = 5;
	startPosition.theta = 0;
	startPosition.ACTIVE = 1;
	poseEnvironment.positionReferences[5].array2D[5][0].x = startPosition.x;
	poseEnvironment.positionReferences[5].array2D[5][0].y = startPosition.y;
	poseEnvironment.positionReferences[5].array2D[5][0].theta = startPosition.theta;
	poseEnvironment.positionReferences[5].array2D[5][0].ACTIVE = startPosition.ACTIVE;
	poseEnvironment.poseActivity[5].array2D[5][0] = startActivation;
	poseEnvironment.maxActivatedCell.x = startPosition.x;
	poseEnvironment.maxActivatedCell.y = startPosition.y;
	poseEnvironment.maxActivatedCell.theta = startPosition.theta;
	poseEnvironment.maxActivatedCell.ACTIVE = startPosition.ACTIVE;

}

////////////////////////////
//                       	//
// Path integration stuff //
//                       	//
////////////////////////////

void pathIntegrateCell(char xP, char yP, char thetaP, char ACTIVEP, char translationX, char translationY)
{
		PoseCellPosition cell;
		cell.x = xP;
		cell.y = yP;
		cell.theta = thetaP;
		cell.ACTIVE = ACTIVEP;
		getActivationDistribution(translationX, translationY, 0);
		for(char relativeX = 0; relativeX < 2; relativeX++)
		{
			char x = getWrappedX(cell.x + relativeX + translationX);

			for(char relativeY = 0; relativeY < 2; relativeY++)
			{
				char y = getWrappedY(cell.y + relativeY + translationY);

				for(char relativeTheta = 0; relativeTheta < 2; relativeTheta++)
				{
					char theta = getWrappedTheta(cell.theta + 0 + relativeTheta);

					poseEnvironment.poseActivity[x].array2D[y][theta] += distribution[relativeX].array2D[relativeY][relativeTheta];
				}
			}
		}
}

void getActivationDistribution(char offsetX, char offsetY, char offsetTheta)
{
	char signX = -1;
	for(char x = 0; x < 2; x++)
	{
		char signY = -1;
		for(char y = 0; y < 2; y++)
		{
			char signTheta = -1;
			for(char theta = 0; theta < 2; theta++)
			{
				double portion = 	((1-x) + signX * offsetX) *	((1-y) + signY * offsetY) *	((1-theta) + signTheta * offsetTheta);
				distribution[x].array2D[y][theta] = portion;

				signTheta = +1;
			}
			signY = +1;
		}
		signX = +1;
	}
}

//this function will be called when a button is pressed then do path integration, exciting etc
void pose3D(char translationX, char translationY)
{
	checkActive();
	for(char i = 0; i < 10; i++)
	{
		for(char j = 0; j < 10; j++)
		{
			for(char k = 0; k < 6; k++)
			{
				PoseCellPosition position3;
				position3.x = poseEnvironment.positionReferences[i].array2D[j][k].x;
				position3.y = poseEnvironment.positionReferences[i].array2D[j][k].y;
				position3.theta = poseEnvironment.positionReferences[i].array2D[j][k].theta;
				position3.ACTIVE = poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE;
				if(position3.ACTIVE)
				{
					pathIntegrateCell(position3.x, position3.y, position3.theta, position3.ACTIVE, translationX, translationY);
				}
			}
		}
	}
	doExcitation(0.02);
	double activeSum = doInhibition(0.02);
	doNormalisation(activeSum);
}

///////////////////////////
//                       //
// Drawing Pose Cells    //
//                       //
///////////////////////////

void displayMaxPoseCell()
{
	eraseDisplay();
	PoseCellPosition position4;
	position4.y = poseEnvironment.maxActivatedCell.y;
	position4.x = poseEnvironment.maxActivatedCell.x;
	int top = (int) position4.y;
	int left = (int) position4.x;
	nxtDisplayString(1, "%d %d", top, left);
	nxtFillRect(left, top + 6, left + 10, top);
}


///////////////////////////
//                       //
// Main part starts here //
//                       //
///////////////////////////


task main()
{
	//button stuff for path integration
	nNxtButtonTask  = -2;
	nNxtExitClicks = 5;
	TButtons nBtn;

	initalisePose();
	eraseDisplay();
	nxtDisplayCenteredTextLine(2, "3D Pose Cells");
  wait10Msec(100);
  displayMaxPoseCell();
  while(true)
  {

		if((nBtn = nNxtButtonPressed) > -1)
		{
			switch(nBtn)
			{
				case kLeftButton: {pose3D(-1,0); displayMaxPoseCell();} break;
				case kRightButton: {pose3D(1,0); displayMaxPoseCell();} break;
				case kEnterButton: {pose3D(0,1); displayMaxPoseCell();} break;
				case kExitButton: {pose3D(0,-1); displayMaxPoseCell();} break;
				default: break;
			}

		}
	}
	//starting pose cell due to robotC not allowing Pose position struct to be returned
}
