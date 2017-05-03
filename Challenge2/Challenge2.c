#pragma config(Sensor, S1,     colorLeft,     sensorEV3_Color)
#pragma config(Sensor, S3,     colorRight,    sensorEV3_Color)
#pragma config(Sensor, S2,     sonar,         sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA, leftMotor,     tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorD, rightMotor,    tmotorEV3_Large, PIDControl, encoder)

#define COLOR_SENSOR_AVERAGE_BUFFER_LEN 10
#define SONAR_SENSOR_AVERAGE_BUFFER_LEN 10

#define MOVING_AVG_WEIGHT 75.0

//used when wandering
#define LOWER_MOVING_AVG_WEIGHT 25.0

//used when reacq
#define REAQ_MVG_WEIGHT 45.0

#define LOW_SPEED_WANDER 40
#define HI_SPEED_WANDER 30

int COLOR_LOWER_BOUND = 5;
int COLOR_UPPER_BOUND = 60;

float avgColorLeft = 0,
			avgColorRight = 0,
      avgDist = 0;

int sonarAvg = 255;


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


float getMovingAvg (float lastAvg, float alpha, int newestReading) {
  return lastAvg + (alpha * (newestReading - lastAvg));
}

void getSonarAvg(float weight) {
	//so we want yet another weight which attempts to reject deviant values
  //we can derive this weight from the newest reading's deviation from the current average
	//This weight should have a minimum amount, should always be positive
  // diff = min ( 1, sonarAvg - sonarVar) never let this be zero to avoid div by zero among other things
  // weight = max ( 1, 10 / diff ); let the final weight here never be greater than one
  // This last statement basically accepts the full weight value of deviations by
	int sonarVar;
	sonarVar = getUSDistance(sonar);

	//float diff = min ( 1, sonarAvg - sonarVar ); //never let this be zero to avoid div by zero among other things
  //float weight = max ( 1, 10 / diff ); //let the final weight here never be greater than one

	sonarAvg = getMovingAvg(sonarAvg, weight / 100.0, sonarVar);
}

void getColorAvg (float weight) {
	long r,g,b;
	getColorRGB(colorLeft, r, g, b);
	float cLeft = ( g + b) / 2.0;


	getColorRGB(colorRight, r, g, b);
	float cRight = (g + b) / 2.0;

	avgColorLeft = getMovingAvg(avgColorLeft,   weight / 100.0, cLeft);
	avgColorRight = getMovingAvg(avgColorRight, weight / 100.0, cRight);
	writeDebugStreamLine("left: %f, right: %f", avgColorLeft, avgColorRight);
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
	writeDebugStreamLine("random walk");
//this loop will let robot walking repeatly but with random pattern.
  while(true) {
  	//generates number between 0 and 99.
  	//starts with 50/50 chance of going left or right.
  	//lower end of probability will always be for turning left.
  	turnProb = rand() % 100;
  	rTime = rand() % 900 + 300;
  	getColorAvg(LOWER_MOVING_AVG_WEIGHT);

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
bool offline (float wt) {
	writeDebugStreamLine("offline");
	getColorAvg(wt);
	return fabs(avgColorLeft - COLOR_LOWER_BOUND) > 3 && fabs(avgColorRight - COLOR_LOWER_BOUND) > 3;
}

//Play the sound when it's offline
void playTheOfflineSound () {
 	writeDebugStreamLine("not acquired");
  playTone(100, 25);
  playTone(300, 25);
  playTone(500, 25);
  playTone(700, 25);
}

//Attempt reacquire the line.
// First, skid steer left. If a sensor acquires black, return true
//second turn right for double the amount
bool acquireLine () {
	writeDebugStreamLine("acquire line");
	bool acquired = false;
	mv(-20, 20);

	for (int i = 0; i < 15 && !acquired; i++) {
		//rotate left about 100
		acquired = acquired || !offline(REAQ_MVG_WEIGHT);

		delay(100);
  }

  mv(20, -20);
  //writeDebugStreamLine("Turning right");
  for (int i = 0; i < 30 && !acquired; i++) {
		//then rotate right 200
    acquired = acquired || !offline(REAQ_MVG_WEIGHT);

		delay(100);
  }

  if (!acquired) {
	  ///TODO extract to function
   playTheOfflineSound();

	  mv(-20, 20);
	  delay(1200);
	} else {
		mv(10, 10);
		for (int i = 0; i < 15; i++) {
			delay(100);
			getColorAvg(LOWER_MOVING_AVG_WEIGHT);
	  }

	  if (offline(LOWER_MOVING_AVG_WEIGHT)) {
	  	acquired = false;
	  	playTheOfflineSound();
	  	//play the not acquired sound
	  }
  }

  return acquired;
}

void setMotorSpeedBasedOnLightSensor() {
	//int baseSpeed = 10;

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
	writeDebugStreamLine("line follow");
	while (true) {
		getColorAvg(LOWER_MOVING_AVG_WEIGHT);
		if (fabs(avgColorLeft - COLOR_LOWER_BOUND) < 3 || fabs(avgColorRight - COLOR_LOWER_BOUND) < 3) {
			stopTask(randomWalk);
			setLEDColor(ledRedFlash);

			int offlineCount = 0;

			while (true) {

				setMotorSpeedBasedOnLightSensor();
				delay(100);
				getColorAvg(MOVING_AVG_WEIGHT);

				if (offline(MOVING_AVG_WEIGHT)) {
					setLEDColor(ledOrangeFlash);
					offlineCount += 1;
			  } else {
			    offlineCount = 0;
			  }


			  if (offlineCount > 5) {
			  	setLEDColor(ledGreenFlash);
			  	bool acquired = acquireLine();

			  	//I'm sorry I'm trying
			  	if (!acquired) {
			  		break;
			  	}
			  }
		  }

			startTask(randomWalk);
		}
  }
}

void calBlack() {
	for (int i = 0; i < 10; i++) {
		getColorAvg(MOVING_AVG_WEIGHT);
	  delay(100);
  }

  COLOR_LOWER_BOUND = ((avgColorLeft + avgColorRight) / 2);
}

void calWhite() {
	for (int i = 0; i < 10; i++) {
		getColorAvg(MOVING_AVG_WEIGHT);
	  delay(100);
  }

  COLOR_UPPER_BOUND = ((avgColorLeft + avgColorRight) / 2);
}

#define STOP_DIST 14

//Priority 0
//Kill line detection to chase object and kill wander
//3 feet is approximately 90 from the getDistance function
task objectDetect () {
	//stopTask(lineFollow);
	bool detected;
	while(true) {

		if (detected) {
			getSonarAvg(50.0);
		} else {
			delay(100);
			getSonarAvg(15.0);
		}

		writeDebugStreamLine("snr %d", sonarAvg);
		if(sonarAvg < 90 && sonarAvg > STOP_DIST) {
			detected = true;
			stopTask(randomWalk);
			stopTask(lineFollow);
			setLEDColor(ledOff);

			float actualSpeed = (sonarAvg / 90.0) * 128;
			writeDebugStreamLine("speed: %f", actualSpeed);
			mv(actualSpeed, actualSpeed);
			delay(10);
		} else if(sonarAvg <= STOP_DIST && detected){
			mv(0, 0);

			wait1Msec(2000);

			mv(-55, -55);

			detected = false;
			sonarAvg = 255;
			wait1Msec(200);
			randomDir();
			startTask(randomWalk);
			delay(100);
			startTask(lineFollow);
			delay(100);
		} else if(sonarAvg > 90 && detected){
			detected = false;
			sonarAvg = 255;
			startTask(randomWalk);
			delay(100);
			startTask(lineFollow);
			delay(100);
		}
	}


	//

}

task main()
{
	setLEDColor(ledOrangeFlash);
	delay(2000);
	calBlack();

	setLEDColor(ledGreenFlash);
  delay(2000);
	calWhite();


	startTask(randomWalk);
	delay(100);
	startTask(lineFollow);

	delay(100);
	startTask(objectDetect);

	//startTask(getColorAvg);

	while (true);

}
