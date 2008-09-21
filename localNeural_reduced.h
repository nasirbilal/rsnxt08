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
int firstUnit = 40; //for neural units
int secondUnit = 80;
int thirdUnit = 120;
int fourthUnit = 160;
int fifthUnit = 200;
const char numLocalCells = 100; //number of possible views
const char numNeuralUnits = 15; //number of neural units representing a view

typedef struct
{
  float localArray[numNeuralUnits];
} localViewCell;


//----Functions----//
void clearTemp();
void setRight(int rightSonarValue);
void setCentre(int centreSonarValue);
void setLeft(int leftSonarValue);
void normaliseTemp();
void setTemp();
float dotMultiply();
void checkLocalCell();
