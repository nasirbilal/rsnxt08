//////this c file just holds math functions useful to use with ratSLAM///////

//get Degrees
int getDegrees(float angle)
{
  return (int) (angle * 180/PI);
}

//ensure only ever within the unit circle
float wrappedDegrees360 (float angle)
{
  while(angle >= 360)
  {
  	angle -= 360;
  }
  while(angle < 0)
  {
  	angle += 360;
  }
  return angle;
}

//Angle between two vectors
float getAngleDegrees(float x, float y)
{
  if(x == 0)
  {
  	if(y >= 0)
  	{
  	  return (90);
  	}
  	else
  	{
  	  return -(90);
  	}
  }
	float theta = getDegrees(atan(y/x));
  if(x < 0)
  {
  	return (theta + 180);
  }
  if(y < 0)
  {
    return (theta + 90);
  }
  return theta;
}

//the rotation between two degrees with the smallest magnitude
float getRotationDegrees(float angle1, float angle2)
{
  float difference = wrappedDegrees360(angle2 - angle1);
  if(difference > 180)
  {
  	difference = difference - 360;
  }
  return difference;
}

//Length of a vector
float getLength(float x, float y)
{
  return (sqrt(x*x + y*y));
}
