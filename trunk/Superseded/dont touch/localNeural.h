////////////////////////////////////////////////////////////////////
//                                                                //
//      Title:-   Local View Header                               //
//      Author:-  Lachlan Smith (RobotC)                          //
//                Mark Wakabayashi (Java version)                 //
//                Michael Milford (original algorithim/c code)    //
//      Purpose:- Implement local view data from sonar sensors    //
//                                                                //
////////////////////////////////////////////////////////////////////

//----Variables----//
int firstUnit = 15; //for neural units
int secondUnit = 45;
int thirdUnit = 75;
int fourthUnit = 105;
int fifthUnit = 135;
int sixthUnit = 195;
const char numNeuralUnits = 18; //number of neural units representing a view

typedef struct
{
  float localArray[18];
} localViewCell;

//----Functions----//
void clearTemp();
void setRight(float rightSonarValue);
void setCentre(float centreSonarValue);
void setLeft(float leftSonarValue);
void normaliseTemp();
void setTemp();
float dotMultiply();
void checkLocalCell();
