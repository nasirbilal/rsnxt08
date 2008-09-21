////////////////////////////////////////////////////////////////////
//                                                                //
//      Title:-   Pose Cell Header                                //
//      Author:-  Lachlan Smith (RobotC)                          //
//                Mark Wakabayashi (Java version)                 //
//                Michael Milford (original algorithim/c code)    //
//      Purpose:- Stuff for pose cells                            //
//                                                                //
////////////////////////////////////////////////////////////////////

//----variables used for pose cells----//
const char sizeX = 10; //number of cells in X dimension
const char sizeY = 10; //number of cells in Y dimension
const char sizeTheta = 6; //number of cells in theta dimension

float startActivation = 0.5; //the starting activation of the first cell
char influenceXY = 1; // the level of influence that cells have on neighbouring cells
char influenceTheta = 1;
float weightVarianceXY = 0.8;//for gaussian distribution stuff
float weightVarianceTheta = 0.5;//for gaussian distribution stuff
char weightScaleFactor = 14; //strength of influence between cells
float stepSize = 0.2; //this is to account for something
float globalInhibition = 0.014;//0.014
char poseEstimationRadius = 6;
float injectionStrength = 0.075;
const int listLength = 600;


//----Structures - mainly 3D arrays----//

//Represents the pose cell
typedef struct {
	char x;
	char y;
	char theta;
} PoseCellPosition;

typedef struct {
  char x;
	char y;
	char theta;
	float cellActivation;
} poseCell;


//matrix that holds data for the excitation matrix
typedef struct {
	float array2D[3][3];
} matrixExcite;

//matrix that holds the data for the distribution of activity
typedef struct {
	float array2D[2][2];
} matrixDistribution;

//the pose environment
typedef struct {
	poseCell poseListOne[listLength];
  poseCell poseListTwo[listLength];
	PoseCellPosition maxActivatedCell;
	float maxCellActivation;
} PoseCellStructure;


//initalise structures
PoseCellStructure poseWorld;
matrixExcite excitation_Weights[3];
matrixDistribution distribution[2];
PoseCellPosition position;


//----Initialises functions----//
void startCell();
void setupPoseStructure();
void excitationMatrixSetup();
char getWrappedX(char indexX);
char getWrappedY(char indexY);
char getWrappedTheta(char indexTheta);
void setActivation(char cellX, char cellY, char cellTheta, float activation);
void injectEnergy (float stepSize, char xCell, char yCell, char thetaCell);
void doExcitation(float stepSize);
float doInhibition(float stepSize);
void doNormalisation(float activationSum);
void getActivationDistribution(float offsetX, float offsetY, float offsetTheta);
void pathIntegrateCell(char xp, char yp, char thetap, float deltaTheta, float translation);
void initialisePose();
void iterate(float stepSize);
void pose3D(float deltaTheta, float translation);
void drawRect(char xCo, char yCo, char percent);
