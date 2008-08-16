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
  long creationTime;
} experience;

experience experienceMap[100];
/*
void setExperience(char id, vector3D odo, vector3D pose, localViewCell local)
{
  memset(experienceMap[id].odoPose,odo,3*sizeof(int));
  memset(experienceMap[id].poseCellsPose, pose,3*sizeof(int));
  memset(experienceMap[id].localView, local,18*sizeof(float));
}
*/

float compareTo(experience &experience1, experience &experience2)
{
  //first test
	float array1[18], array2[18], compareArray[18];
	memset(compareArray, 0, 72);
	memcpy(array1, experience1.localView, 72);
	memcpy(array2, experience2.localView, 72);
	if((array1 == compareArray) || (array1 != array2))
	{
	  	return 0;
	}

	//2nd test
	vector3D thisPose;
	vector3D otherPose;
	memcpy(thisPose, experience1.poseCellsPose, 6);
	memcpy(otherPose, experience2.poseCellsPose, 6);



}

float sum(localViewCell &array)
{
	char x;
	float sum = 0;
  for(x = 0; x < 18; x++)
  {
    sum += array.localArray[x];
  }
  return sum;
}

task main()
{


}
