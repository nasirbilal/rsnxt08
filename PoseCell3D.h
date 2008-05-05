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
//       Rewrite         //
//                       //
///////////////////////////

const char sizeX = 10; // number of cells representing x dimension
const char sizeY = 10; // number of cells representing y dimension
const char sizeTheta = 6; // number of cells representing theta dimension
const float startActivation = 0.5; // the starting activity of the first activity packet
const char influenceXY = 1; // the range of influence of pose cells on neighbouring pose cells (x,y)
const char influenceTheta = 1; // the range of influence of pose cells on neighbouring pose cells (Theta)
const float weightVarianceXY = 0.8; // XY variance in metres for the Gaussian weight matrix
const float weightVarianceTheta = 0.8; // Theta variance in metres for the Gaussian weight matrix
const char weightScaleFactor = 14; // strength of influence between pose cells
const float globalInhibition = 0.014; // the rate of linear decay
const char poseEstimationRadius = 6; // maximum distance of cells to the maximally activated cell included in averaging to estimate the most likely position
const char xyRange = 3; // total range in cells of the xy range
const char thetaRange = 3; // range of the theta dimension
const float stepsize = 0.2; //the step size


///////////////////////////
//                       //
//      Structures       //
//                       //
///////////////////////////

///Note to self:- try and declare all instances of structures etc early so that RobotC is happy

typedef struct {
	char x;
	char y;
	char theta;
	char ACTIVE;
} PoseCellPosition;

typedef struct {
	PoseCellPosition array2D[sizeY][sizeTheta]; //using an array of structures to create [][][]
} matrixPositionReferences;

typedef struct {
	float array2D[sizeY][sizeTheta]; //using an array of structures to create [][][]
} matrixPoseActivity;

typedef struct {
	float array2D[3][3];
} matrixExcite;

typedef struct {
	float array2D[2][2];
} matrixDistribution;


typedef struct {
	matrixPositionReferences positionReferences[sizeX];
	matrixPoseActivity poseActivity[sizeX];
	PoseCellPosition maxActivatedCell;
} PoseCellStructure;

///////////////////////////
//                       //
//    Initialisation     //
//                       //
///////////////////////////

void doExcitation();
void doInhibition();
void doNormalisation();
void setActivition(char i, char j, char k, char ACTIVE, float activation);
void excitationMatrixSetup();
char getWrappedX(char indexX);
char getWrappedY(char indexY);
char getWrappedTheta(char indexTheta);
void setupPoseStructure();
void initialisePose();
void pathIntegrateCell(char xP, char yP, char thetaP, char ACTIVEP, char translationX, char translationY);
void getActivationDistribution(char offsetX, char offsetY, char offsetTheta);
matrixExcite excitation_Weights[3];
PoseCellStructure poseEnvironment;
matrixDistribution distribution[2];
PoseCellPosition position; //for any cell stuff
float activationSum = 0; // for inhibition / normalisation
float inhibition = globalInhibition * stepsize;
