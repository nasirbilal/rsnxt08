/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Title:- Pose Cells source file
//	Author:- Lachlan Smith (RobotC)
//					 Mark Wakabayashi (Jave version)
//					 Michael Milford (original algorithim/c code)
//	Purpose:- The purpose of this program is to implement pose cells and path integration in RobotC for the Lego NXT.  This version will ignore
//					  local view calibration and attempt to display the active pose cell on the NXT display using built in drawing functions in a purely
//					  2d form, only showing a x and y plot and ignoring theta.
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////
//                       //
// Global Variables      //
//                       //
///////////////////////////

//Currently taken straight from ramp code but will probably need some tweaking
//

char sizeX = 10; // number of cells representing x dimension
char sizeY = 10; // number of cells representing y dimension
char sizeTheta = 6; // number of cells representing theta dimension
float startActivation = 0.5; // the starting activity of the first activity packet
char influenceXY = 1; // the range of influence of pose cells on neighbouring pose cells (x,y)
char influenceTheta = 1; // the range of influence of pose cells on neighbouring pose cells (Theta)
float weightVarianceXY = 0.8; // XY variance in metres for the Gaussian weight matrix
float weightVarianceTheta = 0.8; // Theta variance in metres for the Gaussian weight matrix
char weightScaleFactor = 14; // strength of influence between pose cells
float globalInhibition = 0.014; // the rate of linear decay
char poseEstimationRadius = 6; // maximum distance of cells to the maximally activated cell included in averaging to estimate the most likely position
char xyRange = 3; // total range in cells of the xy range
char thetaRange = 3; // range of the theta dimension

///////////////////////////
//                       //
// Structure Definitions //
//                       //
///////////////////////////

//Pose Cell structure//


// PoseCellPosition
typedef struct {
	char x;
	char y;
	char theta;
	char ACTIVE = 0;
} PoseCellPosition;

typedef struct {
	PoseCellPosition array2D[10][6]; //using an array of structures to create [][][]
} matrix3DBigCell;

typedef struct {
	float array2D[10][6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
														0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
														0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
														0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
														0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
														//using an array of structures to create [][][]
} matrix3DBigAct;

typedef struct {
	float array2D[3][3];
} matrix3DSmall;

typedef struct {
	float array2D[2][2];
} matrix3DSmallX2;


typedef struct {
	matrix3DBigCell positionReferences[10];
	matrix3DBigAct poseActivity[10];
	PoseCellPosition maxActivatedCell;
} PoseCellStructure;



void doExcitation(float stepsize);
float doInhibition(float stepSize);
void doNormalisation(float activationSum);
void setActivition(char x, char y, char theta, char ACTIVE, float activation);
void excitationMatrixSetup();
char getWrappedX(char indexX);
char getWrappedY(char indexY);
char getWrappedTheta(char indexTheta);
void setupPoseStructure();
void initalisePose();
void pathIntegrateCell(char xP, char yP, char thetaP, char ACTIVEP, char translationX, char translationY);
void getActivationDistribution(char offsetX, char offsetY, char offsetTheta);
matrix3DSmall excitation_Weights[3];
PoseCellStructure poseEnvironment;
matrix3DSmallX2 distribution[2];

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
