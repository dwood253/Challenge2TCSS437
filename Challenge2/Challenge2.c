#pragma config(Sensor, S1,     colorLeft,     sensorEV3_Color)
#pragma config(Sensor, S3,     colorRight,    sensorEV3_Color)
#pragma config(Sensor, S2,     sonar,         sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA, leftMotor,     tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorD, rightMotor,    tmotorEV3_Large, PIDControl, encoder)

#define COLOR_SENSOR_AVERAGE_BUFFER_LEN 10
#define SONAR_SENSOR_AVERAGE_BUFFER_LEN 10
#define COLOR_LOWER_BOUND 20
#define COLOR_UPPER_BOUND 60


float avgColorLeft = 0,
			avgColorRight = 0,
      avgDist = 0;

void mv (int left, int right) {
	setMotorSpeed(leftMotor, left);
	setMotorSpeed(rightMotor, right);
}

bool allBlack () {
	return avgColorLeft < COLOR_LOWER_BOUND && avgColorRight < COLOR_LOWER_BOUND;
}

bool allWhite () {
	return avgColorLeft > COLOR_UPPER_BOUND && avgColorRight > COLOR_UPPER_BOUND;
}


int getGaussianSum (int num) {
	int sum = 0;
  for (;num > 0; num--) {
		sum += num;
  }
  return sum;
}

float getMemberOfMovingAvg (int i, int value, float gSum) {
	return value * ((i + 1) / gSum);
}

float getMovingAvg (float lastAvg, float alpha, int newestReading) {
  return lastAvg + (alpha * (newestReading - lastAvg));
}

void getColorAvg () {
	avgColorLeft = getMovingAvg(avgColorLeft, 30 / 55.0, getColorReflected(colorLeft));
	avgColorRight = getMovingAvg(avgColorRight, 30 / 55.0, getColorReflected(colorRight));
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
	setLEDColor(ledGreen);

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

bool acquireLine () {
	bool acquired = false;

		//Rotate left until both sensors are on or off

		//If both sensors go off, rotate right, if both are on, proceed straight
	  //Rotate right until both sensors a

		mv(-10, 10);

		for (int i = 0; i < 10; i++) {
			if (allBlack()) {
				acquired = true;
				return true;
		  } else if (allWhite()) {
		    break;
		  }

  writeDebugStreamLine("avg c l: %f, avg c r: %f, avg dist: %f", avgColorLeft, avgColorRight, avgDist);
		  getColorAvg();
			delay(100);
	  }

	  mv(10, -10);

		for (int i = 0; i < 20; i++) {
			if (allBlack()) {
				acquired = true;
				return true;
		  } else if (allWhite()) {
		    break;
		  }

  writeDebugStreamLine("avg c l: %f, avg c r: %f, avg dist: %f", avgColorLeft, avgColorRight, avgDist);
		  getColorAvg();
			delay(100);
	  }
  return acquired;
}

void followLine () {
	bool following = true;

	while (following) {

	}
}

//Priority 1
//Kill wander task to follow line
//Black is approximately 10 +- 5
//White is approximately 70 +- 10
task lineFollow () {
	while (true) {
		if (avgColorLeft < COLOR_LOWER_BOUND || avgColorRight < COLOR_UPPER_BOUND) {
			setLEDColor(ledRedFlash);
			stopTask(randomWalk);
			bool acquired = acquireLine();

  writeDebugStreamLine("avg c l: %f, avg c r: %f, avg dist: %f", avgColorLeft, avgColorRight, avgDist);
			mv(20, 20);

			setLEDColor(ledOrangeFlash);
			delay(1000);

			//followLine();
			//DO the right thing
			startTask(randomWalk);
		}
		getColorAvg();
  }
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

void setupAverages () {
	//initialize left/right color sensor average
	//init distance sensor average
	int colorAvgLeft [COLOR_SENSOR_AVERAGE_BUFFER_LEN];
	int colorAvgRight [COLOR_SENSOR_AVERAGE_BUFFER_LEN];
	int sonarAvg [SONAR_SENSOR_AVERAGE_BUFFER_LEN];

	for (int i = 0; i < COLOR_SENSOR_AVERAGE_BUFFER_LEN; i++) {
		colorAvgLeft[i] = getColorReflected(colorLeft);
		colorAvgRight[i] = getColorReflected(colorRight);
  }

  for (int i = 0; i < SONAR_SENSOR_AVERAGE_BUFFER_LEN; i++) {
		sonarAvg[i] = getUSDistance(sonar);
  }

  //Gauss(10) = 55
  float tmpSum = 0,
  			tmpSum2;

  int gSumColor = getGaussianSum(COLOR_SENSOR_AVERAGE_BUFFER_LEN),
      gSumSonar = getGaussianSum(SONAR_SENSOR_AVERAGE_BUFFER_LEN);

  for (int i = 0; i < COLOR_SENSOR_AVERAGE_BUFFER_LEN; i++) {
  	tmpSum += getMemberOfMovingAvg(i, colorAvgLeft[i], gSumColor);
		tmpSum2 += getMemberOfMovingAvg(i, colorAvgRight[i], gSumColor);
  }
  avgColorLeft = tmpSum;
  avgColorRight = tmpSum2;

  tmpSum = 0;
  for (int i = 0; i < SONAR_SENSOR_AVERAGE_BUFFER_LEN; i++) {
  	tmpSum += getMemberOfMovingAvg(i, sonarAvg[i], gSumSonar);
  }

  avgDist = tmpSum;


  writeDebugStreamLine("avg c l: %f, avg c r: %f, avg dist: %f", avgColorLeft, avgColorRight, avgDist);
}

task main()
{

//	startTask(objectDetect);
	startTask(lineFollow);
	startTask(randomWalk);
	//startTask(getColorAvg);

	while (true);

}
