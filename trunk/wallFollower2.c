#pragma config(Sensor, S1,     leftSonar,           sensorSONAR)
#pragma config(Sensor, S2,     centreSonar,         sensorSONAR)
#pragma config(Sensor, S3,     rightSonar,          sensorSONAR)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int desiredLeft = 15;
int desiredSpeed = 25;

void doLeftTurn()
{
    nSyncedMotors = synchBC;
  	nSyncedTurnRatio = -100;
  	nMotorEncoderTarget[motorB] = 190;
  	motor[motorB] = -desiredSpeed;
  	while(nMotorRunState[motorB] != runStateIdle) {}
    nSyncedMotors = synchNone;
}

void doRightTurn()
{
    nSyncedMotors = synchBC;
  	nSyncedTurnRatio = -100;
  	nMotorEncoderTarget[motorB] = 190;
  	motor[motorB] = desiredSpeed;
  	while(nMotorRunState[motorB] != runStateIdle) {}
    nSyncedMotors = synchNone;
}

void wallFollow()
{
  int leftSonarValue = SensorValue[leftSonar];
  int previousLeftSonarValue = 0;
  nxtDisplayCenteredBigTextLine(1,"%d", leftSonarValue);
  int err = desiredLeft - leftSonarValue;
  nxtDisplayCenteredBigTextLine(4,"%d", err);
  if(leftSonarValue > 30)
  {
    motor[motorB] = 0;
  	motor[motorC] = 0;
  	nSyncedMotors = synchBC;
  	nSyncedTurnRatio = 100;
  	nMotorEncoderTarget[motorB] = 400;
  	motor[motorB] = desiredSpeed;
  	while(nMotorRunState[motorB] != runStateIdle) {}
  	doLeftTurn();
  	nSyncedMotors = synchBC;
  	nSyncedTurnRatio = 100;
  	leftSonarValue = SensorValue[leftSonar];
    while(leftSonarValue > 30)
    {
  	  motor[motorB] = desiredSpeed;
  	  leftSonarValue = SensorValue[leftSonar];
    }
  	nSyncedMotors = synchNone;
  }
  else
  {
    if(err > 4 && err <= 8) //ergo need to increase left motor reduce right motor
    {
  	  motor[motorB] = desiredSpeed+5;
  	  motor[motorC] = desiredSpeed-5;
  	  wait1Msec(50);

  	  motor[motorB] = desiredSpeed-5;
  	  motor[motorC] = desiredSpeed+5;
  	  wait1Msec(25);
  	  //motor[motorB] = desiredSpeed;
  	  //motor[motorC] = desiredSpeed;

  	 }
    else if(err > 8) //ergo need to increase left motor reduce right motor
    {
  	  motor[motorB] = desiredSpeed+10;
  	  motor[motorC] = desiredSpeed-10;
  	  wait1Msec(50);

  	  motor[motorB] = desiredSpeed-10;
  	  motor[motorC] = desiredSpeed+10;
  	  wait1Msec(50);

  	  //motor[motorB] = desiredSpeed;
  	 // motor[motorC] = desiredSpeed;
    }
    else if(err < -4 && err >= -8) //ergo need to increase left motor reduce right motor
    {
  	  motor[motorB] = desiredSpeed-5;
  	  motor[motorC] = desiredSpeed+5;
  	  wait1Msec(50);

  	  motor[motorB] = desiredSpeed+5;
  	  motor[motorC] = desiredSpeed-5;
  	  wait1Msec(25);
  	 // motor[motorB] = desiredSpeed;
  	 // motor[motorC] = desiredSpeed;

  	 }
    else if(err < -8) //ergo need to increase left motor reduce right motor
    {
  	  motor[motorB] = desiredSpeed-10;
  	  motor[motorC] = desiredSpeed+10;
  	  wait1Msec(50);

  	  motor[motorB] = desiredSpeed+10;
  	  motor[motorC] = desiredSpeed-10;
  	  wait1Msec(50);
  	 // motor[motorB] = desiredSpeed;
  	 // motor[motorC] = desiredSpeed;

  	 }
    else
    {
      motor[motorB] = desiredSpeed;
  	  motor[motorC] = desiredSpeed;
    }
  }
}

task main()
{
  nSyncedMotors = synchNone;
  nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
  motor[motorB] = desiredSpeed;
  motor[motorC] = desiredSpeed;
  ClearTimer(T1);
  while(1)
  {
  	int centreSonarValue = SensorValue[centreSonar];
  	if(time100[T1] >= 1)
  	{
  	  wallFollow();
  		ClearTimer(T1);
  	}
  	if(centreSonarValue < 19)
  	{
  		doRightTurn();
  		motor[motorB] = desiredSpeed;
  	  motor[motorC] = desiredSpeed;
  	  ClearTimer(T1);
  	}
  }


}
