#pragma config(Sensor, S1,     colorLeft,         sensorEV3_Color)
#pragma config(Sensor, S3,     colorRight,         sensorEV3_Color)
#pragma config(Sensor, S2,     sonar,         sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          leftMotor,     tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorD,          rightMotor,    tmotorEV3_Large, PIDControl, encoder)


//this is for wondering right
void randomRight() {
	setMotorSpeed(leftMotor, 60);
	setMotorSpeed(rightMotor, 50);
}

//this is for wondering left
void randomLeft() {
	setMotorSpeed(leftMotor, 50);
	setMotorSpeed(rightMotor, 60);
}

//this is the random walking task.
task randomWalk() {
	//varibles that
	long rTime = 0;
	long turnProb = 0;
	long rightProbability = 50;
	long leftProbability = 50;


//this loop will let robot walking repeatly but with random pattern.
  while(true) {
  	//generates number between 0 and 99.
  	//starts with 50/50 chance of going left or right.
  	//lower end of probability will always be for turning left.
  	turnProb = rand() % 100;
  	rTime = rand() % 900 + 300;

	//random turning algorithm.
  	if(turnProb < leftProbability) {
  		randomLeft();
  		rightProbability += 10;
  		leftProbability -= 10;
  	} else {
  		randomRight();
  		rightProbability -= 10;
  		leftProbability += 10;
  	}
		wait1Msec(rTime);
	}
}

task main()
{
	startTask(randomWalk);

	while (true);
}
