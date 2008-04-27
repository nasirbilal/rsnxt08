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

int sizeX = 10; // number of cells representing x dimension
int sizeY = 10; // number of cells representing y dimension
int sizeTheta = 36; // number of cells representing theta dimension
int lengthX = 12.5; // the length of x (in metres) represented by the pose
int lengthY= 12.5; // the length of Y (in metres) represented by the pose
int startActivation = 0.5; // the starting activity of the first activity packet
int influenceXY = 1; // the range of influence of pose cells on neighbouring pose cells (x,y)
int influenceTheta = 1; // the range of influence of pose cells on neighbouring pose cells (Theta)
int weightVarianceXY = 8; // XY variance in metres for the Gaussian weight matrix
int weightVarianceTheta = 8; // Theta variance in metres for the Gaussian weight matrix
int weightScaleFactor = 14; // strength of influence between pose cells
double globalInhibition = 0.014; // the rate of linear decay
int poseEstimationRadius = 6; // maximum distance of cells to the maximally activated cell included in averaging to estimate the most likely position
short xyRange = 3; // total range in cells of the xy range
short thetaRange = 3; // range of the theta dimension

///////////////////////////
//                       //
// Structure Definitions //
//                       //
///////////////////////////

//Pose Cell structure//


// PoseCellPosition
typedef struct {
	int x;
	int y;
	int theta;
	int poseActivity;
	short ACTIVE = 0;
} PoseCellPosition;

typedef struct {
	PoseCellPosition array2D[10][36]; //using an array of structures to create [][][]
} matrix3DBig;

typedef struct {
	double array2D[3][3];
} matrix3DSmall;


typedef struct {
	matrix3DBig positionReferences[10];
	PoseCellPosition maxActivatedCell;
	int numCells = 1;
	int maxNumActive = 1;
} PoseCellStructure;


double getNormalisedGaussian(int n, double variance);
void vectorMultiply(double *array1, double *array2, double *answer, int N);
void getExcitationWeight(int relativeX, int relativeY, int relativeTheta);
void doExcitation(double stepsize);
void getStartCell();
void setupWeightMatrix();
double doInhibition(double stepSize);
void doNormalisation(double activationSum);
