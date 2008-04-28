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
//                              - cell sizes are now smaller, going to try a 10x10x36 instead
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

void excitationMatrixSetup()
{
	excitation_Weights[0].array2D[0][0] = 0.0846;
	excitation_Weights[0].array2D[0][1] = 0.2953;
	excitation_Weights[0].array2D[0][2] = 0.0846;
	excitation_Weights[0].array2D[1][0] = 0.2953;
	excitation_Weights[0].array2D[1][1] = 1.0305;
	excitation_Weights[0].array2D[1][2] = 0.2953;
	excitation_Weights[0].array2D[2][0] = 0.0846;
	excitation_Weights[0].array2D[2][1] = 0.2953;
	excitation_Weights[0].array2D[2][2] = 0.0846;

	excitation_Weights[1].array2D[0][0] = 0.2953;
	excitation_Weights[1].array2D[0][1] = 1.0305;
	excitation_Weights[1].array2D[0][2] = 0.2953;
	excitation_Weights[1].array2D[1][0] = 1.0305;
	excitation_Weights[1].array2D[1][1] = 3.5969;
	excitation_Weights[1].array2D[1][2] = 1.0305;
	excitation_Weights[1].array2D[2][0] = 0.2953;
	excitation_Weights[1].array2D[2][1] = 1.0305;
	excitation_Weights[1].array2D[2][2] = 0.2953;

	excitation_Weights[2].array2D[0][0] = 0.0846;
	excitation_Weights[2].array2D[0][1] = 0.2953;
	excitation_Weights[2].array2D[0][2] = 0.0846;
	excitation_Weights[2].array2D[1][0] = 0.2953;
	excitation_Weights[2].array2D[1][1] = 1.0305;
	excitation_Weights[2].array2D[1][2] = 0.2953;
	excitation_Weights[2].array2D[2][0] = 0.0846;
	excitation_Weights[2].array2D[2][1] = 0.2953;
	excitation_Weights[2].array2D[2][2] = 0.0846;
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
/*
//inhibition stuff
double doInhibition(double stepSize)
{
	double inhibition = globalInhibition * stepSize;

	double activationSum = 0;

	for(char x = 0; x < 10; x++)
	{
		for(char y = 0; y < 10; y++)
		{
			for(char z = 0; z < 6; z++)
			{
				char isActive = poseEnvironment.positionReferences[x].array2D[y][z].ACTIVE;
				if(isActive) //only touch cells that are active - not the best way of doing this but will do for testing
				{
					double activation = poseEnvironment.poseActivity[x].array2D[y][z] - inhibition;
					if(activation <= 0)
					{
					activation = 0;
					}
					//setActivition(position, activation);
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
void doExcitation(double stepsize)
{
	for(char i = 0; i < 10; i++)
	{
		for(char j = 0; j < 10; j++)
		{
			for(char k = 0; k < 6; k++)
			{
				PoseCellPosition position;
				position.x = poseEnvironment.positionReferences[i].array2D[j][k].x;
				position.y = poseEnvironment.positionReferences[i].array2D[j][k].y;
				position.theta = poseEnvironment.positionReferences[i].array2D[j][k].theta;
				position.ACTIVE = poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE;
				if(position.ACTIVE)
				{
					double thisActivation = poseEnvironment.poseActivity[i].array2D[j][k];
					for(char relX = -influenceXY; relX <= influenceXY; relX++)
					{
						char neighbourX = getWrappedX(position.x + relX);
						for(char relY = -influenceXY; relY <= influenceXY; relY++)
						{
							char neighbourY = getWrappedY(position.y + relY);
							for(char relTheta = -influenceTheta; relTheta <= influenceTheta; relTheta++)
							{
								char neighbourTheta = getWrappedTheta(position.theta + relTheta);
								double excitationWeight = excitation_Weights[neighbourX].array2D[neighbourY][neighbourTheta];
								poseEnvironment.poseActivity[neighbourX].array2D[neighbourY][neighbourTheta] += thisActivation * excitationWeight;
							}
						}
					}
				}
			}
		}
	}
}


//using setActivation
void setActivition(PoseCellPosition cell, double activation)
{
	double previousActivation = poseEnvironment.poseActivity[cell.x].array2D[cell.y][cell.theta];

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
		cell.ACTIVE = 0;
	}
	else
	{
		cell.ACTIVE = 1;
	}
	poseEnvironment.poseActivity[cell.x].array2D[cell.y][cell.theta] = activation;
	if(activation > PoseCellStructure.poseActivity[maxPose.x].array2D[maxPose.y][maxPose.theta])
	{
		poseEnvironment.maxActivatedCell.x = cell.x;
		poseEnvironment.maxActivatedCell.y = cell.y;
		poseEnvironment.maxActivatedCell.theta = cell.theta;
	}
}

*/

///////////////////////////
//                       //
// Pose Cell start       //
//                       //
///////////////////////////


//Determine Startcell - currently using a global starting Pose
void initalisePose()
{
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
	excitationMatrixSetup();
}

////////////////////////////
//                       	//
// Path integration stuff //
//                       	//
////////////////////////////

void pathIntegrateCell(PoseCellPosition cell, char translationX, char translationY)
{
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
///////////////////////////
//                       //
// Drawing Pose Cells    //
//                       //
///////////////////////////

void displayMaxPoseCell()
{
	eraseDisplay();
	PoseCellPosition position;
	position.y= poseEnvironment.maxActivatedCell.y;
	position.x = poseEnvironment.maxActivatedCell.x;
	int top = (int) position.y;
	int left = (int) position.x;
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
	initalisePose();
	eraseDisplay();
	nxtDisplayCenteredTextLine(2, "3D Pose Cells");
  wait10Msec(100);
  displayMaxPoseCell();
	wait10Msec(1000);
	//starting pose cell due to robotC not allowing Pose position struct to be returned
 }
