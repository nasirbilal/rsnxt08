/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Title:- Pose Cells
//	Author:- Lachlan Smith (RobotC)
//					 Mark Wakabayashi (Jave version)
//					 Michael Milford (original algorithim/c code)
//	Purpose:- The purpose of this program is to implement pose cells and path integration in RobotC for the Lego NXT.  This version will ignore
//					  local view calibration and attempt to display the active pose cell on the NXT display using built in drawing functions in a purely
//					  2d form, only showing a x and y plot and ignoring theta.
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Things to do:- work out how poseCellPosition and poseCellStructure are going to work together. Essentially want a poseCellStructure containing
//								all the posecellPosition's but need to consider the activity matrix and write matrix.  Only want to implement these once.  Possibly
//					 			in the poseCellStructure have like poseCellPosition holdArray[some number].  Need to think bout it.
//							- use the setActivity function in the rest of the code
//							- continue with posestructure setup
//
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Changes made from orignal code to make this work.
//				- cell sizes are now smaller, going to try a 10x10x36 instead
//				- only going to use int instead of doubles, or even chars.
//				- establish excitation matrix by hand (used matlab)
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

///////////////////////////
//                       //
// Excitatory Matrix stuff //
//                       //
///////////////////////////

//established it like this to try and take some computational strain off NXT
void excitationMatrixSetup()
{
	excitation_Weights[1].array2D[1][1] = 0.0846;
	excitation_Weights[1].array2D[1][2] = 0.2953;
	excitation_Weights[1].array2D[1][3] = 0.0846;
	excitation_Weights[1].array2D[2][1] = 0.2953;
	excitation_Weights[1].array2D[2][2] = 1.0305;
	excitation_Weights[1].array2D[2][3] = 0.2953;
	excitation_Weights[1].array2D[3][1] = 0.0846;
	excitation_Weights[1].array2D[3][2] = 0.2953;
	excitation_Weights[1].array2D[3][3] = 0.0846;

	excitation_Weights[2].array2D[1][1] = 0.2953;
	excitation_Weights[2].array2D[1][2] = 1.0305;
	excitation_Weights[2].array2D[1][3] = 0.2953;
	excitation_Weights[2].array2D[2][1] = 1.0305;
	excitation_Weights[2].array2D[2][2] = 3.5969;
	excitation_Weights[2].array2D[2][3] = 1.0305;
	excitation_Weights[2].array2D[3][1] = 0.2953;
	excitation_Weights[2].array2D[3][2] = 1.0305;
	excitation_Weights[2].array2D[3][3] = 0.2953;

	excitation_Weights[3].array2D[1][1] = 0.0846;
	excitation_Weights[3].array2D[1][2] = 0.2953;
	excitation_Weights[3].array2D[1][3] = 0.0846;
	excitation_Weights[3].array2D[2][1] = 0.2953;
	excitation_Weights[3].array2D[2][2] = 1.0305;
	excitation_Weights[3].array2D[2][3] = 0.2953;
	excitation_Weights[3].array2D[3][1] = 0.0846;
	excitation_Weights[3].array2D[3][2] = 0.2953;
	excitation_Weights[3].array2D[3][3] = 0.0846;
}


void doExcitation(double stepsize)
{


}


///%%%%%%%%%**********As with doExcitation need to look at how the arrays are workin and how they work together
/// note:- need to somehow keep track of active cells rather than searchin through the environment each time but due to array stuff could be easiest way
double doInhibition(double stepSize)
{
	double inhibition = globalInhibition * stepSize;

	double activationSum = 0;

	for(char x = 0; x < 10; x++) //from a 100 array of posePositions
	{
		for(char y = 0; y < 10; y++)
		{
			for(char z = 0; z < 6; z++)
			{
				PoseCellPosition position = poseEnvironment.positionReferences[x].array2D[y][z];
				if(position.ACTIVE) //only touch cells that are active - not the best way of doing this but will do for testing
				{
					double activation = position.poseActivity - inhibition;
					if(activation <= 0)
					{
					activation = 0;
					}
					setActivition(position, activation);
					activationSum += activation;
				}
			}
		}
	}
	return activationSum;
}

void doNormalisation(double activationSum)
{
	for(int x = 0; x < 100; x ++)
	{
		PoseCellPosition position = poseEnvironment[x];
		if(position.ACTIVE)
		{
			position.poseActivity = position.poseActivity / activationSum;
		}
	}
}

//using setActivation as will update the activity matrix as well, which is how i shall keep track of active cells
void setActivition(PoseCellPosition cell, double activation)
{
	double previousActivation = cell.poseActivity;
	if(previousActivation == activation)
	{
		return;
	}
	if(activation == 0)
	{
		cell.ACTIVE = 0;
	}
	cell.poseActivity = activation;
	poseEnvironment.positionReferences[cell.x].array2D[cell.y][cell.theta] = cell;
	if(activation > PoseCellStructure.maxActivatedCell.poseActivity)
	{
		poseEnvironment.maxActivatedCell = cell;
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
	PoseCellPosition startPosition;
	startPosition.x = 5;
	startPosition.y = 5;
	startPosition.theta = 0;
	startPosition.poseActivity = startActivation;
	startPosition.ACTIVE = 1;
	poseEnvironment.positionReferences[5].array2D[5][0] = startPosition;
	poseEnvironment.maxActivatedCell = startPosition;
	excitationMatrixSetup();
}


///////////////////////////
//                       //
// Main part starts here //
//                       //
///////////////////////////


task main()
{
	initalisePose();

	 //starting pose cell due to robotC not allowing Pose position struct to be returned
 }
