//Experience map stuff
//--this structs are still under debate--//
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
  vector3D odoPose;
  vector3D poseCellsPose;
  localViewCell localView;
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

experience experienceMap[5];
experienceLink links[5];

float maxAssociationRadiusXY = 0.35;
char maxAssociationRadiusTheta = 3; //ratioed down due to only 6 degrees in theta
char mapCorrectionRateXY = 1;
char mapCorrectionRateTheta = 1;
int nextLink = 0;

/////////////--------End of Header---------///////////////////
/*
void setExperience(char id, vector3D odo, vector3D pose, localViewCell local)
{
  memset(experienceMap[id].odoPose,odo,3*sizeof(int));
  memset(experienceMap[id].poseCellsPose, pose,3*sizeof(int));
  memset(experienceMap[id].localView, local,18*sizeof(float));
}
*/

void setLink(int startID, int endID)
{


}

////////////////////////////////for checking if need a new experience/////////////////////////////
char compareArray(localViewCell &view1, localViewCell &view2)
{
	float array1[18], array2[18], nullArray[18];
	memset(nullArray, 0, 72);
	memcpy(array1, view1, 72);
	memcpy(array2, view2, 72);

	char x;
	char check = 0;
	for(x = 0; x<18; x++)
	{
		if(array1[x] != nullArray[x])
		{
		  check = 1; // not a null vector
		  break;
		}
	}
	if(check)
	{
		for(x = 0; x<18; x++)
	  {
		  if(array1[x] != array2[x])
		  {
		    return 0; // not a null vector
		  }
		}
		return 1;
	}
	else {return 0;}
}


float compareTo(experience &experience1, experience &experience2)
{
  //first test


	char firstTest = compareArray(experience1.localView,experience2.localView);
	if(!firstTest)
	{
	  return 0;
	}

	//2nd test
	vector3D thisPose;
	vector3D otherPose;
	memcpy(thisPose, experience1.poseCellsPose, 6);
	memcpy(otherPose, experience2.poseCellsPose, 6);
  char thetaAbsDist = abs(thisPose.theta - otherPose.theta);
  if(thetaAbsDist > maxAssociationRadiusTheta)
  {
    return 0;
  }

  //3rd test
  float maxXYDistSquared = maxAssociationRadiusXY * maxAssociationRadiusXY;
  int xyDistSquared = ((otherPose.x - thisPose.x) * (otherPose.x - thisPose.x)) +
                        ((otherPose.y - thisPose.y) * (otherPose.y - thisPose.y));
  if(xyDistSquared > maxXYDistSquared)
  {
    return 0;
  }

  //otherwise is a measure of comparison from 0 to 1 comprised of a 0.5 contribution from theta and xy respectively
  return (2 - (sqrt(xyDistSquared) / maxAssociationRadiusXY) -
                  (thetaAbsDist / maxAssociationRadiusTheta)) * 0.5;

}
//////////////////////////////////end of check if new experience needed/////////////////////////////////////////
task main()
{
  experience EX1;
  experience EX2;
  float aE1[18] = {0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5,1.6,1.7};
  float aE2[18] = {0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5,1.6,1.7};
  float test[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  if(aE1 == test)
  {
    nxtDisplayCenteredBigTextLine(3,"true");
  }
  else{
  	nxtDisplayCenteredBigTextLine(3,"false");
  }

  vector3D aV1;
  vector3D aV2;
  aV1.x = 1; aV1.y = 1; aV1.theta = 1;
  aV2.x = 2; aV2.y = 2; aV2.theta = 2;
  memset(EX1.odoPose,0,6);
  memset(EX2.odoPose,0,6);
  memcpy(EX1.poseCellsPose,aV1,6);
  memcpy(EX2.poseCellsPose,aV2,6);
  memcpy(EX1.localView, aE1, 72);
  memcpy(EX2.localView, aE2, 72);

  float compareValue = compareTo(EX1,EX2);

}
