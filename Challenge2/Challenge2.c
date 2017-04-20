#pragma config(Sensor, S1,     colorLeft,         sensorEV3_Color)
#pragma config(Sensor, S3,     colorRight,         sensorEV3_Color)
#pragma config(Sensor, S2,     sonar,         sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          leftMotor,     tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorD,          rightMotor,    tmotorEV3_Large, PIDControl, encoder)

#define COLOR_SENSOR_AVERAGE_BUFFER_LEN 10
#define SONAR_SENSOR_AVERAGE_BUFFER_LEN 10

float getMovingAvg (float lastAvg, float alpha, int length, int newestReading) {
  return lastAvg + (alpha * (newestReading - lastAvg));
}

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
//Lowest priority 2
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


//Priority 1
//Kill wander task to follow line
//Black is approximately 10 +- 5
//White is approximately 70 +- 10
task lineFollow () {
	stopTask(randomWalk);

	//DO the right thing

	startTask(randomWalk);
}


//Priority 0
//Kill line detection to chase object and kill wander
//3 feet is approximately 90 from the getDistance function
task objectDetect () {
	stopTask(lineFollow);
	stopTask(randomWalk);

	//DO the right thing

	startTask(lineFollow);
	startTask(randomWalk);
}




task main()
{
	//initialize left/right color sensor average
	//init distance sensor average
	int [COLOR_SENSOR_AVERAGE_BUFFER_LEN] colorAvg;
	int [SONAR_SENSOR_AVERAGE_BUFFER_LEN] sonarAvg;
	for (int i = 0; i < COLOR_SENSOR_AVERAGE_BUFFER_LEN; i++) {

  }

  for (i = 0; i < SONAR_SENSOR_AVERAGE_BUFFER_LEN; i++) {

  }


	startTask(objectDetect);
	startTask(lineFollow);
	startTask(randomWalk);

	while (true);
}
