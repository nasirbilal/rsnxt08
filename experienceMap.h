////////////////////////////////////////////////////////////////////
//                                                                //
//      Title:-   Experience Map Header                           //
//      Author:-  Lachlan Smith (RobotC)                          //
//                Mark Wakabayashi (Java version)                 //
//                Michael Milford (original algorithim/c code)    //
//      Purpose:- Implement the Experience Map                    //
//                                                                //
////////////////////////////////////////////////////////////////////

//----Variables----//
float maxAssociationRadiusXY = 0.35;
char maxAssociationRadiusTheta = 3; //ratioed down due to only 6 degrees in theta
char mapCorrectionRateXY = 1;
char mapCorrectionRateTheta = 1;
const int numOfExperiences = 5; //main file

//----Structs----//
typedef struct
{
  int x;
	int y;
	int theta;
} vector3D;

typedef struct
{
  float localArray[18];
} localViewCell;

typedef struct
{
	//the id number of experience used for link stuff - will equal position in array - may not need this
	int ID;
	//where lies on map
	vector3D mapPose; //6 bytes
  //in terms of odometry from encoders
	vector3D odoPose;  //6bytes
  //in terms of pose cells
	vector3D poseCellsPose; //6bytes
  //the local view for this experience
	localViewCell localView; //72 bytes ---->total = 90
	//holds the index for the experienceLink array for all experiences links for this experience to others
	int outLinks[5];  //10
	//holds the index for the experienceLink array for all experiences links for this experience to others
	int inLinks[5]; //10
	//for moment assuming only max of 5 links
} experience;

typedef struct
{
  int startExperienceID; //instead of storing entire experience to save memory will just store where the experiences
  int endExperienceID; //lie in the experienceMap array
	int transitionTime; //time taken for transistion
  int translationDistance; //based on encoder data
  int translationAngle; //relative angle to move
  int rotation; //change in orientation expected after completing link movement
} experienceLink;

typedef struct
{
	int lastMatchedExperienceID; //id of the last matched experience
	int lastExperienceMeanTime;
	experience currentExperience;
	int currentExperienceStartTime;
	experience experienceMap[numOfExperiences];
} experienceMapModule;


//----Functions----//
experienceMapModule Map;
experienceLink links[5];

//sets experience - not sure if i really need this
void setExperience(char id, vector3D &odo, vector3D &pose, localViewCell &local);

void createNewExperience(); //still needs work to make it compatible with poseLocal

void linkLastToCurrent();

void linkExperience(experience &cExperience);

//sets links --------------may need fixing
void setLink(int startID, int endID);

//----Compares arrays - this is due to RobotC unable to do this----//
char compareArray(localViewCell &view1, localViewCell &view2);

//----Compares two Experiences----//
float compareTo(experience &experience1, experience &experience2);

//this compares the current experience to previous experiences returning the closest matching experience
//returning the array id of the closest experience.
int matchExperience(experience &currentExperience);

//----Corrects the Experience Map----//
void mapCorrection();
