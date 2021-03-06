#pragma config(Sensor, S1,     colorLeft,     sensorEV3_Color)
#pragma config(Sensor, S3,     colorRight,    sensorEV3_Color)
#pragma config(Sensor, S2,     sonar,         sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA, leftMotor,     tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorD, rightMotor,    tmotorEV3_Large, PIDControl, encoder)

#define COLOR_SENSOR_AVERAGE_BUFFER_LEN 10
#define SONAR_SENSOR_AVERAGE_BUFFER_LEN 10
#define MOVING_AVG_ALPHA 0.8889

#define LOW_SPEED_WANDER 40
#define HI_SPEED_WANDER 30

int COLOR_LOWER_BOUND = 20;
int COLOR_UPPER_BOUND = 60;

float avgColorLeft = 0,
			avgColorRight = 0,
      avgDist = 0;


//this is making robot to do a random left turn.
void turnLeft() {
	long turnTime = 450;
	setMotorSpeed(leftMotor, -55);
	setMotorSpeed(rightMotor, 55);
	wait1Msec(turnTime);
}

//this is making robot to do a random left turn.
void turnRight() {
	long turnTime = 450;
	setMotorSpeed(leftMotor, 55);
	setMotorSpeed(rightMotor, -55);
	wait1Msec(turnTime);
}

//this will give robot a random direction.
void randomDir() {
	long turnProb = 0;
	turnProb = rand() % 100;
	if(turnProb < 50) {
		turnLeft();
	} else {
		turnRight();
	}
}

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

float getMovingAvg (float lastAvg, int newestReading) {
  return lastAvg + (MOVING_AVG_ALPHA * (newestReading - lastAvg));
}

void getColorAvg () {
	long r,g,b;
	getColorRGB(colorLeft, r, g, b);
	float cLeft = (r + g + b) / 3.0;


	getColorRGB(colorRight, r, g, b);
	float cRight = (r + g + b) / 3.0;

	avgColorLeft = getMovingAvg(avgColorLeft, cLeft);
	avgColorRight = getMovingAvg(avgColorRight, cRight);
	writeDebugStreamLine("avg c l: %f, avg c r: %f, avg dist: %f", avgColorLeft, avgColorRight, avgDist);
}

//this is for wondering right
void randomRight() {
	setMotorSpeed(leftMotor, HI_SPEED_WANDER);
	setMotorSpeed(rightMotor, LOW_SPEED_WANDER);
}

//this is for wondering left
void randomLeft() {
	setMotorSpeed(leftMotor, LOW_SPEED_WANDER);
	setMotorSpeed(rightMotor, HI_SPEED_WANDER);
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

//Are we off the line?
bool offline () {
	getColorAvg();
	return fabs(avgColorLeft - COLOR_LOWER_BOUND) > 10 && fabs(avgColorRight - COLOR_LOWER_BOUND) > 10;
}


bool acquireLine () {
	bool acquired = false;
	mv(-20, 20);
  writeDebugStreamLine("Turning left");
	for (int i = 0; i < 15 && !acquired; i++) {
		//rotate left about 100
		acquired = acquired || !offline();

		delay(100);
  }

  mv(20, -20);
  //writeDebugStreamLine("Turning right");
  for (int i = 0; i < 30 && !acquired; i++) {
		//then rotate right 200
    acquired = acquired || !offline();

		delay(100);
  }

  return acquired;
}

void setMotorSpeedBasedOnLightSensor() {
	int baseSpeed = 10;

	if (avgColorLeft - avgColorRight > 10) {
		//this means go right
	  mv(15, -5);
  } else if (avgColorRight - avgColorLeft > 10) {
  	//go left
  	mv(-5, 15);
  } else {
   	//go straight
		mv(17, 17);
  }

}

//Priority 1
//Kill wander task to follow line
//Black is approximately 10 +- 5
//White is approximately 70 +- 10
task lineFollow () {
	while (true) {

	if (fabs(avgColorLeft - COLOR_LOWER_BOUND) < 10 || fabs(avgColorRight - COLOR_LOWER_BOUND) < 10) {
			setLEDColor(ledRedFlash);
			stopTask(randomWalk);
			bool online = true;
			int offlineCount = 0;

			while (online) {

				setMotorSpeedBasedOnLightSensor();
				delay(100);
				getColorAvg();

				if (offline()) {
					setLEDColor(ledGreen);
					offlineCount += 1;
			  } else {
			    offlineCount = 0;
			  }


			  if (offlineCount > 5) {
			  	setLEDColor(ledGreenFlash);
			  	acquireLine();
			  	break;
			  }

		  }

			//followLine();
			//DO the right thing

			startTask(randomWalk);
		}


		getColorAvg();
  }
}

void calBlack() {
	for (int i = 0; i < 10; i++) {
		getColorAvg();
	  delay(100);
  }

  COLOR_LOWER_BOUND = ((avgColorLeft + avgColorRight) / 2) + 10;
}

void calWhite() {
	for (int i = 0; i < 10; i++) {
		getColorAvg();
	  delay(100);
  }

  COLOR_UPPER_BOUND = ((avgColorLeft + avgColorRight) / 2) - 10;
}

//Priority 0
//Kill line detection to chase object and kill wander
//3 feet is approximately 90 from the getDistance function
task objectDetect () {
	//stopTask(lineFollow);
	int sonarAvg;
	while(true) {
		sonarAvg = getUSDistance(sonar);
		if(sonarAvg < 90 && sonarAvg > 5) {
			stopTask(randomWalk);
			stopTask(lineFollow);
			float actualSpeed = (sonarAvg / 90.0) * 128;
			//writeDebugStreamLine("speed: %f", actualSpeed);
			setMotorSpeed(leftMotor, actualSpeed);
			setMotorSpeed(rightMotor, actualSpeed);
		} else if(sonarAvg <= 5){
			setMotorSpeed(leftMotor, 0);
			setMotorSpeed(rightMotor, 0);
			wait1Msec(2000);
			setMotorSpeed(leftMotor, -55);
			setMotorSpeed(rightMotor, -55);
			wait1Msec(200);
			randomDir();
			startTask(randomWalk);
			startTask(lineFollow);
		} else if(sonarAvg > 95){
			startTask(randomWalk);
			startTask(lineFollow);
		}
	}


	//

}

task main()
{


	setLEDColor(ledGreenFlash);
  delay(2000);
	calWhite();

	setLEDColor(ledOrangeFlash);
	delay(2000);
	calBlack();


  //startTask(objectDetect);
	startTask(lineFollow);
	startTask(randomWalk);
	//startTask(getColorAvg);

	while (true);

}
