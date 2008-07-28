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
char firstUnit = 15; //for neural units
char secondUnit = 45;
char thirdUnit = 75;
char fourthUnit = 105;
char fifthUnit = 135;
char sixthUnit = 195;
const char numLocalCells = 40; //number of possible views
const char numNeuralUnits = 18; //number of neural units representing a view


//----Functions----//
void clearTemp();
void setRight();
void setCentre();
void setLeft();
void normaliseTemp();
void setTemp();
float dotMultiply();
void fillStructArray(char cellNum);
void checkLocalCell();
