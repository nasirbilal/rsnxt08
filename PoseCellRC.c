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

///////////////////////////
//                       //
// Excitory Matrix stuff //
//                       //
///////////////////////////

// Normalised Gaussian for excitory matrix
double getNormalisedGaussian(int n, double variance)
{
	double gaussian[n];
	double x = -0.5 * (n-1);
	double sum = 0;
	for(int i = 0; i < n; i++)
		{
			gaussian[i] = exp(-(x * x) / variance);
			x++;
			sum += gaussian[i];
		}
	for(int i = 0; i < n; i++)
		{
		gaussian[i] = gaussian[i]/sum;

		}
		return gaussian;
}


//ripped straight out of Skynet for the excitory matrix
void vectorMultiply(double *array1, double *array2, double *answer, int N)
{
	int i, j;
	double sum = 0.0, factor;

	for (i = 0; i < N; i++) {
		for (j = 0; j < N; j++) {
			answer = array1 * array2;
			sum += answer;
			answer++;
			array2++;
		}
		array1++;
		array2-=N;
	}

	factor = 1.0 / sum;

	//Now normalize it, first reset the positions within the arrays we are pointing to
	answer -= N * N;

	for (i = 0; i < N; i++) {
		for (j = 0; j < N; j++) {
			answer = answer * factor;
			answer++;
		}
	}
}

void getExcitationWeight(int relativeX, int relativeY, int relativeTheta)
{
	excitation_Weights.i = relativeX + influenceXY;
	excitation_Weights.j = relativeY + influenceXY;
	excitation_Weights.k = relativeTheta + influenceTheta;
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
