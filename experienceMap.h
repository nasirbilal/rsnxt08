////////////////////////////////////////////////////////////////////
//                                                                //
//      Title:-   Experience Map Header                           //
//      Author:-  Lachlan Smith (RobotC)                          //
//                Mark Wakabayashi (Java version)                 //
//                Michael Milford (original algorithim/c code)    //
//      Purpose:- Implement the Experience Map                    //
//                                                                //
////////////////////////////////////////////////////////////////////

//----Variables and Definitions----//
float maxAssociationRadiusXY = 0.35;
char maxAssociationRadiusTheta = 3; //ratioed down due to only 6 degrees in theta
char mapCorrectionRateXY = 1;
char mapCorrectionRateTheta = 1;
const char numOfExperiences = 20; //main file
const char numOfLinksPerExperience = 20; //assuming not many links between various experiences - this could be reduced

//----Structs----//
typedef struct
{
  int x;
	int y;
	int theta;
} vector3D;

typedef struct
{
  char x;
	char y;
	char theta;
} vector3DPose;

typedef struct
{
	//the id number of experience used for link stuff - will equal position in array - may not need this
	int ID;
	//where lies on map
	vector3D mapPose; //6 bytes
  //in terms of odometry from encoders
	vector3D odoPose;  //6bytes
  //in terms of pose cells
	PoseCellPosition poseCellsPose; //3bytes
  //the local view for this experience
	localViewCell localView; //72 bytes ---->total = 90
	//holds the index for the experienceLink array for all experiences links for this experience to others
	int outLinks[numOfLinksPerExperience];  //10
	//holds the index for the experienceLink array for all experiences links for this experience to others
	int inLinks[numOfLinksPerExperience]; //10
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
	int lastMatchedExperienceID; //id of the last matched experience //2
	int lastExperienceMeanTime; //2
	experience currentExperience; //112
	int currentExperienceStartTime; //2
	experience experienceMap[numOfExperiences]; //5*112 = 560
} experienceMapModule;


//----Functions----//
experienceMapModule Map;
experienceLink links[numOfExperiences];

void setExperience(char id, vector3D &odo, vector3D &pose, localViewCell &local);
void createNewExperience(experience &newE);
void setOutlinks(char linkID, experience &startE, experience &endE);
void linkLastToCurrent();
void setLink(int startID, int endID);
char compareArray(localViewCell &view1, localViewCell &view2);
float compareTo(experience &experience1, experience &experience2);
int matchExperience(experience &matchE);
void mapCorrection();
void startUp();
void iterateMap();
void initaliseMap();
