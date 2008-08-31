int influenceXY = 1;
int influenceTheta = 1;

//matrix used for holding pose activity
typedef struct {
	float array2D[10][6]; //using an array of structures to create [][][]
} matrixPoseActivity;

//the pose environment
typedef struct {
	matrixPoseActivity poseAct[10];
} PoseCellStructure;

PoseCellStructure poseWorld;



//----Wrapping values in x-direction in Pose structure----//
char getWrappedX(char indexX)
{
  if(indexX < 0)
  {
    return (indexX + 10);
  }
  else if(indexX >= 10)
  {
    return (indexX - 10);
  }
  return indexX;
}

//----Wrapping values in y-direction in Pose structure----//
char getWrappedY(char indexY)
{
  if(indexY < 0)
  {
    return (indexY + 10);
  }
  else if(indexY >= 10)
  {
    return (indexY - 10);
  }
  return indexY;
}

//----Wrapping values in theta-direction in Pose structure----//
char getWrappedTheta(char indexTheta)
{
  if(indexTheta < 0)
  {
    return (indexTheta + 6);
  }
  else if(indexTheta >= 6)
  {
    return (indexTheta - 6);
  }
  return indexTheta;
}



task main()
{
	memset(poseWorld,0,2400);
	poseWorld.poseAct[5].array2D[5][0] = 0.5;
  int count = 0;
	char i,j,k;
	char relX, relY, relTheta;

  for(i = 0; i < 10; i++)
  {
    for(j = 0; j < 10; j++)
    {
      for(k = 0; k < 6; k++)
      {
      	if(poseWorld.poseAct[i].array2D[j][k] > 0)
      	{
        	float thisActivation = 0;
        	for (relX = -influenceXY; relX <= influenceXY; relX++)
        	{
            char neighbourX = getWrappedX(i + relX);
            for(relY = - influenceXY; relY<= influenceXY; relY++)
            {
              char neighbourY = getWrappedY(j + relY);
              for(relTheta = -influenceTheta; relTheta <= influenceTheta; relTheta++)
              {
              	count += 1;
                char neighbourTheta = getWrappedTheta(k + relTheta);
                nxtDisplayCenteredTextLine(2,"%d",neighbourX);
                nxtDisplayCenteredTextLine(3,"%d",neighbourY);
                nxtDisplayCenteredTextLine(4,"%d",neighbourTheta);
                 nxtDisplayCenteredTextLine(6,"%d",count);
                wait10Msec(100);
              }
            }
          }
        }
      }
    }
  }
}
