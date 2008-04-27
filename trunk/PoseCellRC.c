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
matrix3DSmall excitation_Weights;
PoseCellStructure poseEnvironment;

///////////////////////////
//                       //
// Excitory Matrix stuff //
//                       //
///////////////////////////

// Normalised Gaussian for excitory matrix
double getNormalisedGaussian(double variance)
{
	double gaussian[3];
	double x = -0.5 * (3-1);
	double sum = 0;
	for(int i = 0; i < 3; i++)
		{
			gaussian[i] = exp(-(x * x) / variance);
			x++;
			sum += gaussian[i];
		}
	for(int i = 0; i < 3; i++)
		{
		gaussian[i] = gaussian[i]/sum;

		}
		return gaussian;
}


//ripped straight out of Skynet for the excitory matrix
double vectorMultiply(double array1, double array2, int N)
{ //N is the length of the array - in this case it will be 3, thus the result matrix has the dimensions already entered
	double sum = 0.0;
	double result[3][3];

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			result[i][j] = array1[i] * array2[j];
			sum += result[i][j];
		}
	}

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			result[i][j] /= sum;
		}
	}
	return result;
}


//this will set up the excitory matrix
void setupWeightMatrix()
{
	/*
	Idea for doing 3d matrices
	typedef struct {
		double matrix[3][3];
	} twoDMatrix

	then:-
	twoDMatrix threeDMatrix[3];

	therefore threeDMatrix[x].matrix[y][z]
	*/

	double sum = 0.0;
	double factor = 0.0;
	double a, b, c;

	a = getNormalisedGaussian(weightVarianceXY);
	c = getNormalisedGaussian(weightVarianceTheta);
	b = vectorMultiply(a,a,3);

	for(int x = 0; x < xyRange; x++)
	{
		for(int y = 0; y < xyRange; y++)
		{
			for(int z = 0; z < thetaRange; z++)
			{
				ExciteMatrix[x].matrix2D[y][z] = b[x][y] * c[z];
				sum += ExciteMatrix[x].matrix2D[y][z];
			}
		}

	}

	for(int x = 0; x < xyRange; x++)
	{
		for(int y = 0; y < xyRange; y++)
		{
			for(int z = 0; z < thetaRange; z++)
			{
				ExciteMatrix[x].matrix2D[y][z] = (ExciteMatrix[x].matrix2D[y][z])/sum;
				ExciteMatrix[x].matrix2D[y][z] = (ExciteMatrix[x].matrix2D[y][z])* weightScaleFactor;
			}
		}

	}

}

void doExcitation(double stepsize)
{
	writeMatrix ExciteWriteMatrix = poseEnvironment.Write_Matrix;
	for(int x = 0; x < poseEnvironment.maxNumActive; x++)
	{
### look at carefully

	}

}


///%%%%%%%%%**********As with doExcitation need to look at how the arrays are workin and how they work together
/// note:- need to somehow keep track of active cells rather than searchin through the environment each time but due to array stuff could be easiest way
double doInhibition(double stepSize)
{
	double inhibition = globalInhibition * stepSize;

	double activationSum = 0;

	for(int x = 0; x < 100; x++) //from a 100 array of posePositions
	{
		PoseCellPosition position = poseEnvironment[x];
		if(position.ACTIVE)
		{
			double activation = position.poseActivity - inhibition;
			if(activation <= 0)
			{
				activation = 0;
				position.ACTIVE = 0;
			}
			position.poseActivity = activation;
			activationSum += activation;
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
void setActivition (PoseCellPosition cell, double activation)
{
	double previousActivation = cell.poseActivity;
	if(previousActivation == activation)
	{
		return;
	}
	poseEnvironment.positionReferences[cell.x].array2D[cell.y][cell.theta] = activation;
	if(PoseCellStructure.maxActivatedCell == null || activation > PoseCellStructure.maxActivatedCell.poseActivity)
	{
		PoseCellStructure.maxActivatedCell = cell;
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
	startPosition.x = 25;
	startPosition.y = 25;
	startPosition.theta = 0;
	startPosition.poseActivity = startActivation;
	startPosition.ACTIVE = 1;
	poseEnvironment.positionReferences[25].array2D[25][0] = startPosition;
}


///////////////////////////
//                       //
// Main part starts here //
//                       //
///////////////////////////


task main()
{

	 //starting pose cell due to robotC not allowing Pose position struct to be returned
 }
