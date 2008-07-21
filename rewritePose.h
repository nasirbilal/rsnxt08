///header file///
//variables used for pose cells
const char sizeX = 10; //number of cells in X dimension
const char sizeY = 10; //number of cells in Y dimension
const char sizeTheta = 6; //number of cells in theta dimension
const char lengthX = 5; //length (x) represented by pose
const char lengthY = 5; //length (y) represented by pose
//note theta thought to represent 360 degrees
float startActivation = 0.5; //the starting activation of the first cell
char influenceXY = 1; // the level of influence that cells have on neighbouring cells
char influenceTheta = 1;
float weightVarianceXY = 0.8;//for gaussian distribution stuff
float weightVarianceTheta = 0.5;//for gaussian distribution stuff
char weightScaleFactor = 14; //strength of influence between cells
float stepSize = 0.2; //this is to account for something
float globalInhibition = 0.05;//0.014
char poseEstimationRadius = 6;
//int numActive = 0;

//-------------------------------------------------------------------------//
//  structures
//
// These structure as due to the inability of robotC to handle 3d arrays
//
//-------------------------------------------------------------------------//

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

//just holds a one or a zero to represent if the pose cell is active
typedef struct {
	char array2D[sizeY][sizeTheta]; //using an array of structures to create [][][] //this will be one if active, zero if not
} matrixPositionReferences;

//the pose environment
typedef struct {
	matrixPositionReferences positionReferences[sizeX];
	matrixPoseActivity poseActivity[sizeX];
	PoseCellPosition maxActivatedCell;
} PoseCellStructure;

//initalise structures
PoseCellStructure poseWorld;
matrixPoseActivity tempPoseActivity[sizeX]; //this is used for calculations on the activity of poses
matrixExcite excitation_Weights[3];
matrixDistribution distribution[2];
PoseCellPosition position;

//-------------------------------------------------------------------------//
//  Functions
//
// These structure as due to the inability of robotC to handle 3d arrays
//
//-------------------------------------------------------------------------//

//initialises the start cell and sets starting activation
void startCell();
//initalises all pose cells as inactive and sets all pose activity to zero
void setupPoseStructure();
//due to robotC inability to let one struct variable = another of the same sort we do it the long way
void fillTempPose();
void fillFinalPose();
/*
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
*/
void excitationMatrixSetup();
char getWrappedX(char indexX);
char getWrappedY(char indexY);
char getWrappedTheta(char indexTheta);
//sets the activity in the structure and also sets maximum cell
void setActivation(char cellX, char cellY, char cellTheta, float activation);
//this excites the maxActivated cell and surrounding cells
void doExcitation(float stepSize);
//inhibits surrounding cells;
float doInhibition(float stepSize);
//normalises the activity in pose structure after excitation and inhibition
void doNormalisation(float activationSum);
void getActivationDistribution(float offsetX, float offsetY, float offsetTheta);
//this function handles path integration
void pathIntegrateCell(char xp, char yp, char thetap, float deltaTheta, float translation);
void initialisePose();
void iterate(float stepSize);
void pose3D(float deltaTheta, float translation);
