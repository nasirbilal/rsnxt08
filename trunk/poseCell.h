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

char startActivation = 0.5; //the starting activation of the first cell
char influenceXY = 1; // the level of influence that cells have on neighbouring cells
char influenceTheta = 1;
float weightVarianceXY = 0.8;//for gaussian distribution stuff
float weightVarianceTheta = 0.5;//for gaussian distribution stuff
char weightScaleFactor = 14; //strength of influence between cells
float stepSize = 0.2; //this is to account for something
float globalInhibition = 0.014;//0.014
char poseEstimationRadius = 6;
float injectionStrength = 0.075;
int numberOfCells = 4*sizeTheta*sizeX*sizeY;

char fiftyPercent[36] = {0,1,0,1,0,1,1,0,1,0,1,0,0,1,0,1,0,1,1,0,1,0,1,0,0,1,0,1,0,1,1,0,1,0,1,0};
char seventyFivePercent[36] = {1,1,0,1,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,0,1,1,1,1,0,1,1};
char twentyfivePercent[36] = {0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,1,0,1,0,0,1,0,1,0,0,1,0,0};
char fifteenPercent[36] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0};
char thirtyThreePercent[36] = {0,0,0,0,1,0,0,1,0,1,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,0,1,0,0,0,0,1,0,1,0,0};
char sixtySevenPercent[36] = {1,1,0,1,1,1,1,0,1,0,1,0,0,1,1,1,0,1,1,0,1,1,1,0,0,1,0,1,0,1,1,1,1,0,1,1};
char eightyFivePercent[36] = {1,0,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,0,1};
char twoPercent[36] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


//----Structures - mainly 3D arrays----//

//Represents the pose cell
typedef struct {
	char x;
	char y;
	char theta;
} PoseCellPosition;

//matrix used for holding pose activity
typedef struct {
	float array2D[sizeY][sizeTheta]; //using an array of structures to create [][][]
} matrixPoseActivity;

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
	matrixPoseActivity poseActivity[sizeX];
	PoseCellPosition maxActivatedCell;
} PoseCellStructure;

typedef struct {
	int localView[15];
  char xCell;
  char yCell;
  char thetaCell;
} cellAssociation;

//initalise structures
PoseCellStructure poseWorld;
matrixExcite excitation_Weights[3];
matrixDistribution distribution[2];
matrixPoseActivity tempPose[sizeX];
PoseCellPosition position;
cellAssociation poseAssoc[70];

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
