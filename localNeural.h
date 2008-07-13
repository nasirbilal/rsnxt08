
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Title:-   Local View Header
//      Author:-  Lachlan Smith (RobotC)
//                Mark Wakabayashi (Java version)
//                Michael Milford (original algorithim/c code)
//      Purpose:- Implement local view data from sonar sensors
//
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Changes:- (1st Version)
//
//
//
//
//
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////
//                       //
//      Variables        //
//                       //
///////////////////////////
char firstUnit = 15; //15
char secondUnit = 45; //45
char thirdUnit = 75; //75
char fourthUnit = 105; //105
char fifthUnit = 135;
char sixthUnit = 195;
const char numLocalCells = 40;
const char numNeuralUnits = 18;

///////////////////////////
//                       //
//      Structure        //
//                       //
///////////////////////////

typedef struct
{
//used to store normalised localTemp arrays that are 'new' and dont already exist.
	float localCellTemp[numNeuralUnits];
} localCell;


///////////////////////////
//                       //
//      Functions        //
//                       //
///////////////////////////

void clearTemp();
void setRight();
void setCentre();
void setLeft();
void normaliseTemp();
void setTemp();
float dotMultiply();
void fillStructArray(char cellNum);
void checkLocalCell();
localCell localCellStruct[numLocalCells];

///////////////////////////
//                       //
// Code Used for Testing //
//                       //
///////////////////////////

/*
 nxtDrawLine(5, 20, 94, 20);
    nxtDrawLine(86, 0, 86, 63);
    nxtDrawLine(26, 0, 26, 63);
    nxtDrawLine(56, 0, 56, 63);
    //left
    nxtDisplayStringAt(0, 60, "%1.2f",localTemp[0]);
    nxtDisplayStringAt(0, 50, "%1.2f",localTemp[1]);
    nxtDisplayStringAt(0, 40, "%1.2f",localTemp[2]);
    nxtDisplayStringAt(0, 30, "%1.2f",localTemp[3]);

    //centre
    nxtDisplayStringAt(30, 60, "%1.2f",localTemp[4]);
    nxtDisplayStringAt(30, 50, "%1.2f",localTemp[5]);
    nxtDisplayStringAt(30, 40, "%1.2f",localTemp[6]);
    nxtDisplayStringAt(30, 30, "%1.2f",localTemp[7]);

    //right
    nxtDisplayStringAt(60, 60, "%1.2f",localTemp[8]);
    nxtDisplayStringAt(60, 50, "%1.2f",localTemp[9]);
    nxtDisplayStringAt(60, 40, "%1.2f",localTemp[10]);
    nxtDisplayStringAt(60, 30, "%1.2f",localTemp[11]);

    //TEXT
    nxtDisplayStringAt(88, 60, "N1");
    nxtDisplayStringAt(88, 50, "N2");
    nxtDisplayStringAt(88, 40, "N3");
    nxtDisplayStringAt(88, 30, "N4");
    nxtDisplayStringAt(13, 10, "L");
    nxtDisplayStringAt(40, 10, "C");
    nxtDisplayStringAt(69, 10, "R");
    wait1Msec(1000);
    //eraseDisplay();

    //nxtDisplayCenteredTextLine(1,"Normalised");
    //wait1Msec(100);
    //eraseDisplay();

    normaliseTemp();
    nxtDisplayStringAt(0, 60, "%1.2f",localTemp[0]);
    nxtDisplayStringAt(0, 50, "%1.2f",localTemp[1]);
    nxtDisplayStringAt(0, 40, "%1.2f",localTemp[2]);
    nxtDisplayStringAt(0, 30, "%1.2f",localTemp[3]);

    //centre
    nxtDisplayStringAt(30, 60, "%1.2f",localTemp[4]);
    nxtDisplayStringAt(30, 50, "%1.2f",localTemp[5]);
    nxtDisplayStringAt(30, 40, "%1.2f",localTemp[6]);
    nxtDisplayStringAt(30, 30, "%1.2f",localTemp[7]);

    //right
    nxtDisplayStringAt(60, 60, "%1.2f",localTemp[8]);
    nxtDisplayStringAt(60, 50, "%1.2f",localTemp[9]);
    nxtDisplayStringAt(60, 40, "%1.2f",localTemp[10]);
    nxtDisplayStringAt(60, 30, "%1.2f",localTemp[11]);

    //TEXT
    nxtDisplayStringAt(88, 60, "N1");
    nxtDisplayStringAt(88, 50, "N2");
    nxtDisplayStringAt(88, 40, "N3");
    nxtDisplayStringAt(88, 30, "N4");
    nxtDisplayStringAt(13, 10, "L");
    nxtDisplayStringAt(40, 10, "C");
    nxtDisplayStringAt(69, 10, "R");

    wait1Msec(3000);
    eraseDisplay();
*/
