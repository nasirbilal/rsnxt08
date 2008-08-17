//Experience map stuff
//--this structs are still under debate--//

//includes and main variables
#include "math.c"
#include "experienceMap.h"
int nextID = 0; //main file
int nextLink = 0;//main file
experience tempExperience;

//sets experience - not sure if i really need this
void setExperience(char id, vector3D &odo, vector3D &pose, localViewCell &local)
{
	Map.experienceMap[id].ID = id;
  memcpy(Map.experienceMap[id].odoPose,odo,6);
  memcpy(Map.experienceMap[id].poseCellsPose, pose,6);
  memcpy(Map.experienceMap[id].localView, local,72);
}

void createNewExperience()
{
	int temp[5] = {-1,-1,-1,-1,-1};
	tempExperience.mapPose.x = -1;
	tempExperience.ID = nextID;
	memcpy(tempExperience.outLinks,temp,10);
  memcpy(tempExperience.inLinks,temp,10);
	//memcpy(tempExperience.odoPose,encoderData,6);
	//memcpy(tempExperience.poseCellsPose,maxActivatedCell.pose,6);
	//memcpy(tempExperience.localView,localTemp,72);
  nextID++;
}

//Due to various information being needed to calculate the link it is necessary to create the temp experience
// 'currentExperience' and insert it into the experience map once a new experience becomes active
// the linkLastToCurrent performs the linking - with the linkExperience calling that and setting up a new temp
// experience
void linkLastToCurrent()
{
  float currentMeanTime = (Map.currentExperienceStartTime + nPgmTime) / 2;

  if(Map.lastMatchedExperienceID != -1) //therefore not null
  {
  	//time taken for transistion
  	float transitionTime = currentMeanTime - Map.lastExperienceMeanTime;

  	//Set up pose info and copy data into
    vector3D currentOdoPose;
    memcpy(currentOdoPose, Map.currentExperience.odoPose,6); // may need to memcpy
    vector3D lastOdoPose;
    memcpy(lastOdoPose,Map.experienceMap[Map.lastMatchedExperienceID].odoPose,6); //same as above

    //angle to current (angle = atan(y/x))
    float angleToCurrent;
    angleToCurrent = getAngleDegrees((currentOdoPose.x - lastOdoPose.x),(currentOdoPose.y - lastOdoPose.y));
    float translationAngle = lastOdoPose.theta - angleToCurrent;

    //translation distance (in encoder clicks)
    int translationDistance = getLength((currentOdoPose.x - lastOdoPose.x),(currentOdoPose.y - lastOdoPose.y));

    //relative change in rotation
    float rotation = getRotationDegrees(lastOdoPose.theta, currentOdoPose.theta);

    //therefore has not been inserted onto experience map yet
    if(Map.currentExperience.mapPose.x == -1) //null statement
    {
      vector3D lastMapPose;
      vector3D newMapPose;
      memcpy(lastMapPose, Map.experienceMap[Map.lastMatchedExperienceID].mapPose, 6);

      float calcsAngle = lastMapPose.theta + translationAngle;
      newMapPose.x = (lastMapPose.x + cosDegrees(calcsAngle) * translationDistance);
      newMapPose.y = (lastMapPose.y + sinDegrees(calcsAngle) * translationDistance);
      newMapPose.theta = (lastMapPose.theta + rotation);
      memcpy(Map.currentExperience.mapPose, newMapPose, 6); //set up mapPose for current Experience
      memcpy(Map.experienceMap[Map.currentExperience.ID], Map.currentExperience, 110); //put currentExperience on the map
    }
    char y;
    int linkNumber;
    for(y = 0; y<5; y++)
    {
      linkNumber = Map.experienceMap[Map.lastMatchedExperienceID].outLinks[y];
    	if(linkNumber != -1)
    	{
    	  if(links[linkNumber].endExperienceID == Map.currentExperience.ID)
    	  {
    	    break; //using linkNumber
    	  }
    	}
    	linkNumber = -1;
    }
    //if no link
    if(linkNumber == -1)
    {
    	//create new link with all data needed
      experienceLink link;
    	link.startExperienceID = Map.lastMatchedExperienceID;
    	link.endExperienceID = Map.currentExperience.ID;
    	link.transitionTime = transitionTime;
    	link.translationAngle = translationAngle;
    	link.translationDistance = translationDistance;
    	link.rotation = rotation;
    	memcpy(links[nextLink],link,12);
    	nextLink++; //increment new link
    }
    //else if link
    else
    {
      experienceLink previous;
      memcpy(previous,links[linkNumber],12);
      previous.transitionTime = (previous.transitionTime + transitionTime) / 2;
      previous.translationAngle = (previous.translationAngle + translationAngle) / 2;
      previous.translationDistance = (previous.translationDistance + translationDistance) / 2;
      previous.rotation = (previous.rotation + rotation) / 2;
      memcpy(links[linkNumber], previous,12);
    }
  }

  //set current as previous experience
  Map.lastMatchedExperienceID = Map.currentExperience.ID;
  Map.lastExperienceMeanTime = currentMeanTime;
}


void linkExperience(experience &cExperience)
{
  linkLastToCurrent();
	memcpy(Map.currentExperience,cExperience,110); //need to check this works may have to use memcpy
	Map.currentExperienceStartTime = (int) (nPgmTime/1000); //in seconds
}


//sets links --------------may need fixing
void setLink(int startID, int endID)
{
  links[nextLink].startExperienceID = startID;
  links[nextLink].endExperienceID = endID;
}

//----Compares arrays - this is due to RobotC unable to do this----//
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

//----Compares two Experiences----//
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

//this compares the current experience to previous experiences returning the closest matching experience
//returning the array id of the closest experience.
int matchExperience(experience &currentExperience)
{
  float maxScore = 0;
  int closestMatch = 0;
	char x;
	for(x = 0; x<numOfExperiences; x++)
	{
		float score = compareTo(currentExperience,Map.experienceMap[x]);
		if(score > maxScore)
		{
		  	closestMatch = x;
		  	maxScore = score;
		}

	}
	return closestMatch;
}

//----Corrects the Experience Map----//
void mapCorrection()
{
	//still to come





}



task main()
{


}
