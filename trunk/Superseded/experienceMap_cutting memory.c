//Experience map stuff
//--this structs are still under debate--//

//includes and main variables
#include "math.h"
#include "experienceMap.h"
int nextID = 0; //main file
int nextLink = 0;//main file
vector3D encoderData;
vector3DPose maxActivatedCellPose;
localViewCell localTempView;
TFileHandle hFileHandle = 0;
TFileIOResult nIoResult;
short nFileSize = 1000;
const string sFileName = "experiences.dat";

void createNewExperience(experience &newE)
{
  //going to assume everytime a createNewExperience is called it will affect the Map.currentExperience experience cell
	memset(newE,0,112);
	char temp[5] = {-1,-1,-1,-1,-1}; //My null symbol
	newE.mapPose.x = -1; //null - not yet set up
	newE.ID = nextID;
	memcpy(newE.outLinks,temp,10); //initalise to null
  memcpy(newE.inLinks,temp,10);
	memcpy(newE.odoPose,encoderData,6);
	memcpy(newE.poseCellsPose,maxActivatedCellPose,3);
	memcpy(newE.localView,localTempView,36);
}

//----Sets Outlinks and Inlinks----//
void setOutlinks(char linkID, experience &startE, experience &endE)
{
	char t; //for loop
	for(t = 0; t<numOfLinksPerExperience; t++)
	{
	  if(startE.outLinks[t] == -1)//an empty link
	  {
	  	startE.outLinks[t] = linkID;
	  	break;
	  }
	}
	for(t = 0; t<numOfLinksPerExperience; t++)
	{
		if(endE.inLinks[t] == -1)//an empty link
	  {
	  	endE.inLinks[t] = linkID;
	  	break;
	  }

	}
}

//Due to various information being needed to calculate the link it is necessary to create the temp experience
// 'currentExperience' and insert it into the experience map once a new experience becomes active
// the linkLastToCurrent performs the linking - with the linkExperience calling that and setting up a new temp
// experience
void linkLastToCurrent() //note:- trying char for now to try and cut memory problems
{
  int currentMeanTime = (Map.currentExperienceStartTime + (nPgmTime/1000)) / 2;

  if(Map.lastMatchedExperienceID != -1) //therefore not null
  {
  	//time taken for transistion
  	char transitionTime = currentMeanTime - Map.lastExperienceMeanTime; //shouldnt be more than 128 seconds for transistion

  	//Set up pose info and copy data into
    vector3D currentOdoPose;
    memcpy(currentOdoPose, Map.currentExperience.odoPose,6); // may need to memcpy
    vector3D lastOdoPose;
    memcpy(lastOdoPose,Map.experienceMap[Map.lastMatchedExperienceID].odoPose,6); //same as above

    //angle to current (angle = atan(y/x))
    int angleToCurrent = getAngleDegrees((currentOdoPose.x - lastOdoPose.x),(currentOdoPose.y - lastOdoPose.y));
    int translationAngle = lastOdoPose.theta - angleToCurrent;

    //translation distance (in encoder clicks)
    int translationDistance = getLength((currentOdoPose.x - lastOdoPose.x),(currentOdoPose.y - lastOdoPose.y));

    //relative change in rotation
    int rotation = getRotationDegrees(lastOdoPose.theta, currentOdoPose.theta);

    //therefore has not been inserted onto experience map yet
    if(Map.currentExperience.mapPose.x == -1) //null statement
    {
      int calcsAngle = Map.experienceMap[Map.lastMatchedExperienceID].mapPose.theta + translationAngle;
      Map.currentExperience.mapPose.x = (Map.experienceMap[Map.lastMatchedExperienceID].mapPose.x + cosDegrees(calcsAngle) * translationDistance);
      Map.currentExperience.mapPose.y = (Map.experienceMap[Map.lastMatchedExperienceID].mapPose.y + sinDegrees(calcsAngle) * translationDistance);
      Map.currentExperience.mapPose.theta = (Map.experienceMap[Map.lastMatchedExperienceID].mapPose.theta + rotation);
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
    	links[nextLink].startExperienceID = Map.lastMatchedExperienceID;
    	links[nextLink].endExperienceID = Map.currentExperience.ID;
    	links[nextLink].transitionTime = transitionTime;
    	links[nextLink].translationAngle = translationAngle;
    	links[nextLink].translationDistance = translationDistance;
    	links[nextLink].rotation = rotation;
    	setOutlinks(nextLink, Map.experienceMap[Map.lastMatchedExperienceID],Map.experienceMap[Map.currentExperience.ID]); //sets the outlinks and inlinks
    	nextLink++; //increment new link
    }
    //else if link existes, update
    else
    {
      links[linkNumber].transitionTime = (links[linkNumber].transitionTime + transitionTime) / 2;
      links[linkNumber].translationAngle = (links[linkNumber].translationAngle + translationAngle) / 2;
      links[linkNumber].translationDistance = (links[linkNumber].translationDistance + translationDistance) / 2;
      links[linkNumber].rotation = (links[linkNumber].rotation + rotation) / 2;
    }
  }

  //set current as previous experience
  Map.lastMatchedExperienceID = Map.currentExperience.ID;
  Map.lastExperienceMeanTime = currentMeanTime;
}


void linkExperience(experience &cExperience)
{
  linkLastToCurrent();
	memcpy(Map.currentExperience, cExperience, 112); //add experience
  Map.currentExperienceStartTime = (int) (nPgmTime/1000); //in seconds
}

//----Compares arrays - this is due to RobotC unable to do this----//
char compareArray(localViewCell &view1, localViewCell &view2)
{
	int array1[18], array2[18], nullArray[18];
	memset(nullArray, 0, 36);
	memcpy(array1, view1, 36);
	memcpy(array2, view2, 36);

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
int matchExperience(experience &matchE)
{
  float maxScore = 0;
  int closestMatch = -1;
	char x;
	for(x = 0; x<nextID; x++)
	{
		float score = compareTo(matchE,Map.experienceMap[x]);
		if(score > maxScore)
		{
		  	closestMatch = x;
		  	maxScore = score;
		}

	}
	return closestMatch;
}

//----Corrects the Experience Map----//
//So far have used floats but as all structs use ints i may be able to get away with not using floats at all. Yay to the memory savings
void mapCorrection()
{
	float mapCorrectionXY = mapCorrectionRateXY * 0.5; //could set this at start to be 0.5 and save maybe
	float mapCorrectionTheta = mapCorrectionRateTheta * 0.5;

	experience startExperience;
	experience endExperience;
	vector3D startPose;
	vector3D endPose;
	experienceLink link;

	char z; //for loop
	for(z = 0; z <(nextID-1); z++)
	{
		memcpy(startExperience,Map.experienceMap[z],110); //copy experience being manipulated into startExperience
		memcpy(startPose, startExperience.mapPose, 6); //copy mapPose being manipulated into startPose
    char y; //for loop
    for(y = 0; y < numOfLinksPerExperience; y++)
    {
      if(startExperience.outLinks[y] != -1)
      {
        memcpy(link,links[startExperience.outLinks[y]],12);
        memcpy(endExperience,Map.experienceMap[link.endExperienceID],110);
        memcpy(endPose, endExperience.mapPose, 6);

        //expected position of the end experience
        int angleToTargetEnd = startPose.theta + link.translationAngle;
        int targetEndX = startPose.x + link.translationDistance * cosDegrees(angleToTargetEnd);
        int targetEndY = startPose.y + link.translationDistance * sinDegrees(angleToTargetEnd);

        //expected orientation of the end experience
        int targetEndAngle = startPose.theta + link.rotation;

        //Calulate the 'error' between expected and actual position of end experience
        int xError = targetEndX - endPose.x;
        int yError = targetEndY - endPose.y;
        int thetaError = getRotationDegrees(endPose.theta,targetEndAngle);

        //calculate the adjustment to be made for start and end poses
        int xAdjustment = xError * mapCorrectionXY;
        int yAdjustment = yError * mapCorrectionXY;
        int thetaAdjustment = thetaError * mapCorrectionTheta;

        //Apply adjustments then copy back over previous experiences
        startPose.x -= xAdjustment;
        startPose.y -= yAdjustment;
        startPose.theta = wrappedDegrees360(startPose.theta - thetaAdjustment);

        endPose.x += xAdjustment;
        endPose.y += yAdjustment;
        endPose.theta = wrappedDegrees360(endPose.theta + thetaAdjustment);

        memcpy(startExperience.mapPose, startPose, 6);
        memcpy(Map.experienceMap[z], startExperience, 110);

        memcpy(endExperience.mapPose, endPose, 6);
        memcpy(Map.experienceMap[link.endExperienceID],endExperience,110);
      }
      else {break;} //leave loop faster as there are no more links
    }
  }
}

//----creates the first experience----//
void startUp()
{
	experience startUpExperience;
  createNewExperience(startUpExperience);
  startUpExperience.mapPose.x = 0;
  startUpExperience.mapPose.y = 0;
  startUpExperience.mapPose.theta = 0;
  memcpy(Map.currentExperience,startUpExperience,112);
  Map.currentExperienceStartTime = (int) (nPgmTime/1000);
  //this is my addition - i couldn't find in the java code where the first current view was create placed on the experience map
  memcpy(Map.experienceMap[nextID],Map.currentExperience,112);
  Map.lastMatchedExperienceID = 0;
  nextID++;
}

void iterateMap(float stepSize)
{
  experience newExperience;
  createNewExperience(newExperience);
  int closestExperience = matchExperience(newExperience);

  if(closestExperience != -1)
  {
    if(closestExperience != nextID)
    {
    	experience closeExperience;
    	memcpy(closestExperience,Map.experienceMap[closestExperience],112);
    	linkExperience(closeExperience);
    }
  }
  else
  {
    linkExperience(newExperience);
    nextID++;
  }
}

void initaliseMap()
{
	memset(Map,0,678);
	Map.lastMatchedExperienceID = -1;
	memset(Links,0,60);
  memset(encoderData,0,6);
  memset(maxActivatedCellPose,0,3);
  memset(localTempView,0,35);
  Delete(sfilename, nIoResult);
}
/*
void writeExperiences()
{
	OpenWrite(  hFileHandle, nIoResult, sFileName, nFileSize);
	if (nIoResult != ioRsltSuccess)
	PlaySound(soundLowBuzz);
	else
	{
		int eID;
		vector3D eMapPose;
		vector3D eOdoPose;
		vector3D ePoseCellsPose;
		localViewCell eLocalView;
		int eOutLinks[5];
		int eInLinks[5];
		char nIndex;
		for (nIndex = 0; nIndex < nextID; nIndex++)
		{
			memcpy(eID, Map.experienceMap[nIndex].ID,2);
      memcpy(eMapPose, Map.experienceMap[nIndex].mapPose,6);
      memcpy(eOdoPose, Map.experienceMap[nIndex].odoPose,6);
      memcpy(ePoseCellsPose, Map.experienceMap[nIndex].poseCellsPose,6);
      memcpy(eLocalView, Map.experienceMap[nIndex].localView,72);
      memcpy(eOutLinks, Map.experienceMap[nIndex].outLinks,10);
      memcpy(eInLinks, Map.experienceMap[nIndex].inLinks,10);

      WriteString(hFileHandle, nIoResult, "ID");
      WriteShort(hFileHandle, nIoResult, eID);
      WriteString(hFileHandle, nIoResult, "mapPose");
      WriteShort(hFileHandle, nIoResult, eMapPose.x);
      WriteShort(hFileHandle, nIoResult, eMapPose.y);
      WriteShort(hFileHandle, nIoResult, eMapPose.theta);
      WriteString(hFileHandle, nIoResult, "odoPose");
      WriteShort(hFileHandle, nIoResult, eOdoPose.x);
      WriteShort(hFileHandle, nIoResult, eOdoPose.y);
      WriteShort(hFileHandle, nIoResult, eOdoPose.theta);
      WriteString(hFileHandle, nIoResult, "poseCellsPose");
      WriteShort(hFileHandle, nIoResult, ePoseCellsPose.x);
      WriteShort(hFileHandle, nIoResult, ePoseCellsPose.y);
      WriteShort(hFileHandle, nIoResult, ePoseCellsPose.theta);
      WriteString(hFileHandle, nIoResult, "Local View");
      char x;
      for(x = 0; x<18; x++)
      {
        WriteFloat(hFileHandle, nIoResult, eLocalView.localArray[x]);
      }
      WriteString(hFileHandle, nIoResult, "outLinks");
      for(x = 0; x<5; x++)
      {
        WriteShort(hFileHandle, nIoResult, eOutLinks[x]);
      }
      WriteString(hFileHandle, nIoResult, "inLinks");
      for(x = 0; x<5; x++)
      {
        WriteShort(hFileHandle, nIoResult, eInLinks[x]);
      }
      WriteString(hFileHandle, nIoResult, "End \n");
			if (nIoResult != ioRsltSuccess)
			{
				PlaySound(soundLowBuzz);
				break;
			}
		}
	}
	Close(hFileHandle, nIoResult);



}
*/
task main()
{//testing time bitch
  initaliseMap();
  encoderData.x = 0;
  encoderData.y = 0;
  encoderData.theta = 0;
  maxActivatedCellPose.x = 5;
  maxActivatedCellPose.y = 5;
  maxActivatedCellPose.theta = 0;
  int tempArray[18] = {50,50,0,0,0,0,50,50,0,0,0,0,50,50,0,0,0,0};
  memcpy(localTempView,tempArray,36);
  startUp();
  encoderData.x = 100;
  encoderData.y = 100;
  encoderData.theta = 20;
  maxActivatedCellPose.x = 6;
  maxActivatedCellPose.y = 5;
  maxActivatedCellPose.theta = 0;
  int tempArray2[18] = {60,40,0,0,0,0,50,50,0,0,0,0,50,50,0,0,0,0};
  memcpy(localTempView, tempArray2, 36);
  iterateMap(1);
  encoderData.x = 200;
  encoderData.y = 200;
  encoderData.theta = 0;
  maxActivatedCellPose.x = 7;
  maxActivatedCellPose.y = 6;
  maxActivatedCellPose.theta = 0;
  int tempArray3[18] = {40,40,0,0,0,0,40,40,0,0,0,0,40,40,0,0,0,0};
  memcpy(localTempView, tempArray3, 36);
  iterateMap(1);
  encoderData.x = 300;
  encoderData.y = 300;
  encoderData.theta = 0;
  maxActivatedCellPose.x = 8;
  maxActivatedCellPose.y = 7;
  maxActivatedCellPose.theta = 0;
  int tempArray4[18] = {30,70,0,0,0,0,60,40,0,0,0,0,30,70,0,0,0,0};
  memcpy(localTempView, tempArray4, 36);
  iterateMap(1);

}
