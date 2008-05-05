/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Title:- Pose Cells
//      Author:- Lachlan Smith (RobotC)
//                                       Mark Wakabayashi (Jave version)
//                                       Michael Milford (original algorithim/c code)
//      Purpose:- The purpose of this program is to implement pose cells and path integration in RobotC for the Lego NXT.  This version will ignore
//                                        local view calibration and attempt to display the active pose cell on the NXT display using built in drawing functions in a purely
//                                        2d form, only showing a x and y plot and ignoring theta.
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

//%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%
//%% Things to do.
//%%  - fix path integration
//%%  - test firstly with just running in background - no visualisation
//%%  - visualisation
//%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%

///////////////////////////
//                       //
//       Rewrite         //
//                       //
///////////////////////////

#pragma platform(NXT)

///////////////////////////
//                       //
// Included Files        //
//                       //
///////////////////////////

#include "PoseCell3D.h"

/////////////////////////////
//                         //
// Excitatory Matrix stuff //
//                         //
/////////////////////////////

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
//    Setup Pose Cell      //
//                         //
/////////////////////////////

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
				poseEnvironment.positionReferences[i].array2D[j][k].x = i;
				poseEnvironment.positionReferences[i].array2D[j][k].y = j;
				poseEnvironment.positionReferences[i].array2D[j][k].theta = k;
				poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE = 0;
				poseEnvironment.poseActivity[i].array2D[j][k] = 0;
			}
		}
	}
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

////////////////////////////////////
//                                //
//      Check if Cell Active      //
//                                //
////////////////////////////////////

void checkActive()
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
				if(poseEnvironment.poseActivity[i].array2D[j][k] > 0)
				{
					poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE = 1;
				}
			}
		}
	}
}

////////////////////////////
//                        //
//     Set Activation     //
//                        //
////////////////////////////

void setActivition(char i, char j, char k, char ACTIVE, float activation)
{
	float previousActivation;
	char maxX;
	char maxY;
	char maxTheta;

	previousActivation = poseEnvironment.poseActivity[i].array2D[j][k];
	maxX = poseEnvironment.maxActivatedCell.x;
	maxY = poseEnvironment.maxActivatedCell.y;
	maxTheta = poseEnvironment.maxActivatedCell.theta;

	if(previousActivation == activation)
	{
		return;
	}
	else if(activation <= 0)
	{
		ACTIVE = 0;
	  poseEnvironment.poseActivity[i].array2D[j][k] = 0;
	  return;
	}
	else
	{
		poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE = 1;
		poseEnvironment.poseActivity[i].array2D[j][k] = activation;
	  return;
	}
}

/////////////////////////
//                     //
//    Find Maximum     //
//                     //
/////////////////////////

void findMaximum()
{
	//loop
	char i;
	char j;
	char k;

	//max activated cell stuff
	char mX;
	char mY;
  char mTheta;
	float currentMaximum;
	float tempMaximum;

	mX = poseEnvironment.maxActivatedCell.x;
	mY = poseEnvironment.maxActivatedCell.y;
	mTheta = poseEnvironment.maxActivatedCell.theta;
	currentMaximum = poseEnvironment.poseActivity[mX].array2D[mY][mTheta];
	tempMaximum = 0.0;

	for(i = 0; i < sizeX; i++)
	{
		for(j = 0; j < sizeY; j++)
		{
			for(k = 0; k < sizeTheta; k++)
			{
				tempMaximum = poseEnvironment.poseActivity[i].array2D[j][k];
				if(tempMaximum > currentMaximum)
				{
					currentMaximum = tempMaximum;
					mX = i;
					mY = j;
					mTheta = k;
				}
			}
		}
	}
  poseEnvironment.maxActivatedCell.x = mX;
  poseEnvironment.maxActivatedCell.y = mY;
  poseEnvironment.maxActivatedCell.theta = mTheta;
}



//////////////////////////
//                      //
//      Inhibition      //
//                      //
//////////////////////////

float doInhibition()
{
	char i;
	char j;
	char k;
	float inhibition = globalInhibition * stepsize;
	float activationSum = 0;
  float activation;

	for(i = 0; i < sizeX; i++)
	{
		for(j = 0; j < sizeY; j++)
		{
			for(k = 0; k < sizeTheta; k++)
			{
				position.x = poseEnvironment.positionReferences[i].array2D[j][k].x;
				position.y = poseEnvironment.positionReferences[i].array2D[j][k].y;
				position.theta = poseEnvironment.positionReferences[i].array2D[j][k].theta;
				position.ACTIVE = poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE;
				if(position.ACTIVE) //only touch cells that are active - not the best way of doing this but will do for testing
				{
					activation = poseEnvironment.poseActivity[i].array2D[j][k] - inhibition;
					if(activation <= 0)
					{
					activation = 0;
					}
					setActivition(position.x, position.y, position.theta, position.ACTIVE, activation);
					activationSum += activation;
				}
			}
		}
	}
	return activationSum;
}

//////////////////////////
//                      //
//    Normalisation     //
//                      //
//////////////////////////

void doNormalisation(float activationSum)
{
	char i;
	char j;
	char k;
	char isActive;

	for(i = 0; i < sizeX; i++)
	{
		for(j = 0; j < sizeY; j++)
		{
			for(k = 0; k < sizeTheta; k++)
			{
				isActive = poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE;
				if(isActive)
				{
					poseEnvironment.poseActivity[i].array2D[j][k] /= activationSum;
				}
			}
		}
	}
}

///////////////////////
//                   //
//    Excitation     //
//                   //
///////////////////////

void doExcitation()
{
	//1st loop
	char i;
	char j;
	char k;

	//2nd loop
	char relX;
	char relY;
	char relTheta;

	// Neighbour stuff
	char neighbourX;
	char neighbourY;
	char neighbourTheta;

	float thisActivation;
	float excitationWeight;

	for(i = 0; i < sizeX; i++)
	{
		for(j = 0; j < sizeY; j++)
		{
			for(k = 0; k < sizeTheta; k++)
			{
				position.x = poseEnvironment.positionReferences[i].array2D[j][k].x;
				position.y = poseEnvironment.positionReferences[i].array2D[j][k].y;
				position.theta = poseEnvironment.positionReferences[i].array2D[j][k].theta;
				position.ACTIVE = poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE;
				if(position.ACTIVE)
				{
					thisActivation = poseEnvironment.poseActivity[i].array2D[j][k];
					for(relX = -influenceXY; relX <= influenceXY; relX++)
					{
						neighbourX = getWrappedX(position.x + relX);
						for(relY = -influenceXY; relY <= influenceXY; relY++)
						{
							neighbourY = getWrappedY(position.y + relY);
							for(relTheta = -influenceTheta; relTheta <= influenceTheta; relTheta++)
							{
								neighbourTheta = getWrappedTheta(position.theta + relTheta);
								excitationWeight = (float) excitation_Weights[relX + influenceXY].array2D[relY + influenceXY][relTheta + influenceTheta] * stepsize;
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
//   Path Integration    //
//                       //
///////////////////////////

void pathIntegrateCell(char xP, char yP, char thetaP, char ACTIVEP, char translationX, char translationY)
{
	//loop
	char relativeX;
	char relativeY;
	char relativeTheta;

	//variables
	char x;
	char y;
	char theta;

	// pose cell
	position.x = xP;
	position.y = yP;
	position.theta = thetaP;
	position.ACTIVE = ACTIVEP;
	getActivationDistribution(translationX, translationY, 0);
	for(relativeX = 0; relativeX < 2; relativeX++)
	{
		x = getWrappedX(position.x + relativeX + translationX);

		for(relativeY = 0; relativeY < 2; relativeY++)
		{
			y = getWrappedY(position.y + relativeY + translationY);
			for(relativeTheta = 0; relativeTheta < 2; relativeTheta++)
			{
		  	theta = getWrappedTheta(position.theta + 0 + relativeTheta);
		  	poseEnvironment.poseActivity[x].array2D[y][theta] += distribution[relativeX].array2D[relativeY][relativeTheta];
			}
		}
	}
}


///////////////////////////////
//                           //
//  Activation Distribution  //
//                           //
///////////////////////////////


void getActivationDistribution(char offsetX, char offsetY, char offsetTheta)
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
				portion = 	((1-i) + signX * offsetX) *	((1-j) + signY * offsetY) *	((1-k) + signTheta * offsetTheta);
				distribution[i].array2D[j][k] = portion;

				signTheta = +1;
			}
			signY = +1;
		}
		signX = +1;
	}
}

//////////////////////////////
//                          //
//    Initial Pose Stuff    //
//                          //
//////////////////////////////

void initialisePose()
{
	char startX = sizeX/2;
	char startY = sizeY/2;
	excitationMatrixSetup();
	setupPoseStructure();
	position.x = startX;
	position.y = startY;
	position.theta = 0;
	position.ACTIVE = 1;
	poseEnvironment.positionReferences[startX].array2D[startY][0].x = position.x;
	poseEnvironment.positionReferences[startX].array2D[startY][0].y = position.y;
	poseEnvironment.positionReferences[startX].array2D[startY][0].theta = position.theta;
	poseEnvironment.positionReferences[startX].array2D[startY][0].ACTIVE = position.ACTIVE;
	poseEnvironment.poseActivity[startX].array2D[startY][0] = startActivation;
	poseEnvironment.maxActivatedCell.x = position.x;
	poseEnvironment.maxActivatedCell.y = position.y;
	poseEnvironment.maxActivatedCell.theta = position.theta;
	poseEnvironment.maxActivatedCell.ACTIVE = position.ACTIVE;

}

//////////////////////////////////////
//                                  //
//   Function that runs everthing   //
//                                  //
//////////////////////////////////////

void pose3D(char translationX, char translationY)
{
	//loop
	char i;
	char j;
	char k;

	//variables
	float activeSum;

	//correct any cells that are active and shouldn't be
	checkActive();

	for(i = 0; i < 10; i++)
	{
		for(j = 0; j < 10; j++)
		{
			for(k = 0; k < 6; k++)
			{
				position.x = poseEnvironment.positionReferences[i].array2D[j][k].x;
				position.y = poseEnvironment.positionReferences[i].array2D[j][k].y;
				position.theta = poseEnvironment.positionReferences[i].array2D[j][k].theta;
				position.ACTIVE = poseEnvironment.positionReferences[i].array2D[j][k].ACTIVE;
				if(position.ACTIVE)
				{
					pathIntegrateCell(position.x, position.y, position.theta, position.ACTIVE, translationX, translationY);
				}
			}
		}
	}
	checkActive();
	doExcitation();
	activeSum = doInhibition();
	doNormalisation(activeSum);
	findMaximum();
}


//////////////////////////////
//                          //
//   Visualise pose stuff   //
//                          //
//////////////////////////////
//Need to work on using whole screen

void displayMaxPoseCell()
{
	eraseDisplay();
	position.y = poseEnvironment.maxActivatedCell.y;
	position.x = poseEnvironment.maxActivatedCell.x;
	position.theta = poseEnvironment.maxActivatedCell.theta;
	char top = position.y;
	char left = position.x;
	char angle = position.theta;
	float maxActivation = poseEnvironment.poseActivity[left].array2D[top][angle];
	nxtDisplayString(1, "%d, %d", top, left);
	nxtDisplayString(2, "= %f", maxActivation);
	nxtFillRect(left, top + 6, left + 10, top);
}

//////////////
//          //
//   Main   //
//          //
//////////////

task main()
{
	//button stuff for path integration
	nNxtButtonTask  = -2;
	nNxtExitClicks = 5;
	TButtons nBtn;

	initialisePose();
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

}
