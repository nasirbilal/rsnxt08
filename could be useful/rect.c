char fiftyPercent[36] = {0,1,0,1,0,1,1,0,1,0,1,0,0,1,0,1,0,1,1,0,1,0,1,0,0,1,0,1,0,1,1,0,1,0,1,0};
char seventyFivePercent[36] = {1,1,0,1,1,1,1,1,1,0,1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,0,1,1,1,1,0,1,1};
char twentyfivePercent[36] = {0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,1,0,1,0,0,1,0,1,0,0,1,0,0};
char fifteenPercent[36] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0};
char thirtyThreePercent[36] = {0,0,0,0,1,0,0,1,0,1,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,0,1,0,0,0,0,1,0,1,0,0};
char sixtySevenPercent[36] = {1,1,0,1,1,1,1,0,1,0,1,0,0,1,1,1,0,1,1,0,1,1,1,0,0,1,0,1,0,1,1,1,1,0,1,1};
char eightyFivePercent[36] = {1,0,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,0,1};

void drawRect(char xCo, char yCo, char percent)
{
  char startX = xCo * 6 + 2;
  char startY = yCo * 6 + 2;
  if(percent == 100)
  {
    nxtFillRect(startX, startY, startX + 5, startY + 5);
  }
  else if(percent == 85)
  {
  	int count = 0;
    char yOffset = 0;
    char xOffset = 0;
    while(count<6)
    {
      char x;

      for(x = 0; x<6; x++)
      {
      if(eightyFivePercent[x+xOffset]==1)
      {
         nxtSetPixel(startX+x, startY + yOffset);
        }

      }
      xOffset += 6;
      yOffset+=1;
      count+=1;
    }

  }
  else if(percent == 75)
  {
    int count = 0;
    char yOffset = 0;
    char xOffset = 0;
    while(count<6)
    {
      char x;

      for(x = 0; x<6; x++)
      {
      if(seventyFivePercent[x+xOffset]==1)
      {
         nxtSetPixel(startX+x, startY + yOffset);
        }

      }
      xOffset += 6;
      yOffset+=1;
      count+=1;
    }
  }
  else if(percent == 67)
  {
  	int count = 0;
   char yOffset = 0;
   char xOffset = 0;
   while(count<6)
   {
      char x;

      for(x = 0; x<6; x++)
      {
      if(sixtySevenPercent[x+xOffset]==1)
      {
         nxtSetPixel(startX+x, startY + yOffset);
        }

      }
      xOffset += 6;
      yOffset+=1;
      count+=1;
    }
  }
  else if(percent == 50)
  {
   int count = 0;
   char yOffset = 0;
   char xOffset = 0;
   while(count<6)
   {
      char x;

      for(x = 0; x<6; x++)
      {
      if(fiftyPercent[x+xOffset]==1)
      {
         nxtSetPixel(startX+x, startY + yOffset);
      }

      }
      xOffset += 6;
      yOffset+=1;
      count+=1;
    }
  }
  else if(percent == 33)
  {
  	int count = 0;
    char yOffset = 0;
    char xOffset = 0;
    while(count<6)
    {
      char x;

      for(x = 0; x<6; x++)
      {
      if(thirtyThreePercent[x+xOffset]==1)
      {
         nxtSetPixel(startX+x, startY + yOffset);
      }

      }
      xOffset += 6;
      yOffset+=1;
      count+=1;
    }
  }
  else if(percent == 25)
  {
  	int count = 0;
    char yOffset = 0;
    char xOffset = 0;
    while(count<6)
    {
      char x;

      for(x = 0; x<6; x++)
      {
        if(twentyfivePercent[x+xOffset]==1)
        {
           nxtSetPixel(startX+x, startY + yOffset);
        }
      }
      xOffset += 6;
      yOffset+=1;
      count+=1;
    }
  }
  else if(percent == 15)
  {
  	int count = 0;
    char yOffset = 0;
    char xOffset = 0;
    while(count<6)
    {
      char x;

      for(x = 0; x<6; x++)
      {
        if(fifteenPercent[x+xOffset]==1)
        {
          nxtSetPixel(startX+x, startY + yOffset);
        }
      }
      xOffset += 6;
      yOffset+=1;
      count+=1;
    }
  }
}



task main()
{
///each pose cell is 6 by 6
//to determine the starting co-ord for each pose
//pose x,y (in zero coord that is 0-9) *6 + 2 in each direction
//end of rect is starting plus 5
    nxtDrawRect(1,1,62,62);

    drawRect(5,5,15);

    wait10Msec(400);
}
