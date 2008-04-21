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
twoDMatrix ExciteMatrix[3]; //the 3d matrix holding the excitatory weights
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
	writeMatrix ExciteWriteMatrix = PoseCellStructure.WriteMatrix;
	### continue

}

///////////////////////////
//                       //
// Pose Cell start       //
//                       //
///////////////////////////


//Determine Startcell - currently using a global starting Pose
void getStartCell()
{
	startPosition.x = lengthX;
	startPosition.y = lengthY;
	startPosition.theta = sizeTheta;
}


///////////////////////////
//                       //
// Main part starts here //
//                       //
///////////////////////////


task main()
{
  PoseCellPosition startPosition; //starting pose cell due to robotC not allowing Pose position struct to be returned
  excitationWeights excitation_Weights;

}
