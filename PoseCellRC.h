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

int sizeX = 50; // number of cells representing x dimension
int sizeY = 50; // number of cells representing y dimension
int sizeTheta = 36; // number of cells representing theta dimension
double lengthX = 12.5; // the length of x (in metres) represented by the pose
double lengthY= 12.5; // the length of Y (in metres) represented by the pose
double startActivation = 0.5; // the starting activity of the first activity packet
int influenceXY = 1; // the range of influence of pose cells on neighbouring pose cells (x,y)
int influenceTheta = 1; // the range of influence of pose cells on neighbouring pose cells (Theta)
double weightVarianceXY = 0.8; // XY variance in metres for the Gaussian weight matrix
double weightVarianceTheta = 0.5; // Theta variance in metres for the Gaussian weight matrix
int weightScaleFactor = 14; // strength of influence between pose cells
double globalInhibition = 0.014; // the rate of linear decay
int poseEstimationRadius = 6; // maximum distance of cells to the maximally activated cell included in averaging to estimate the most likely position
short int xyRange = 3; // total range in cells of the xy range
short int thetaRange = 3; // range of the theta dimension


///////////////////////////
//                       //
// Structure Definitions //
//                       //
///////////////////////////

//Pose Cell structure
//Taken from RAMP

typedef struct {
	double i[50]; //sizeX
	double j[50]; //sizeY
  double k[36]; //sizeTheta
} writeMatrix;

typedef struct {
	double i[50]; //sizeX
	double j[50]; //sizeY
  double k[36]; //sizeTheta
} activationMatrix;

// PoseCellPosition
typedef struct {
	double x;
	double y;
	double theta;
} PoseCellPosition;

typedef struct {
	double matrix2D[3][3];
} twoDMatrix;

typedef struct {
	activationMatrix ActivationMatrix;
	writeMatrix WriteMatrix;
	double cellLengthX;
	double cellLengthY;
	double cellLengthTheta;
	PoseCellPosition maxActivatedCell;
	int numCells;
	int maxNumActive;
	double maxActivity;
	int peakIndex;
} PoseCellStructure;



// Excitation weight array - RobotC will not support more than a 2d array.  The solution? try a structure
typedef struct {
	double i[3]; //Note:- these are determined by influenceXY/influenceTheta * 2 + 1
	double j[3]; //As RobotC doesn't like empty arrays these will be set to three
	double k[3];
} excitationWeights;


double getNormalisedGaussian(int n, double variance);
void vectorMultiply(double *array1, double *array2, double *answer, int N);
void getExcitationWeight(int relativeX, int relativeY, int relativeTheta);
void doExcitation(double stepsize);
void getStartCell();
void setupWeightMatrix();
