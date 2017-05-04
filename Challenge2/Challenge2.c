#pragma config(Sensor, S1, colorLeft, sensorEV3_Color)
#pragma config(Sensor, S3, colorRight, sensorEV3_Color)
#pragma config(Sensor, S2, sonar, sensorEV3_Ultrasonic)
#pragma config(Motor, motorA, leftMotor, tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor, motorD, rightMotor, tmotorEV3_Large, PIDControl, encoder)

//This is used for the initial calibration. It was initially thought that this would be used in more instances
//This was not the case, but there's no sense in removing it now.

#define COLOR_SENSOR_AVERAGE_BUFFER_LEN 10
#define SONAR_SENSOR_AVERAGE_BUFFER_LEN 10

//The weight of new values when following a line. A higher weight allows for more precise line following.

#define MOVING_AVG_WEIGHT 75.0

//Weight used when wandering. This weight is lower in order to ignore wrinkles contacting the light sensor.

#define LOWER_MOVING_AVG_WEIGHT 25.0

//Needed a middling weight for reacquiring lines. This turned out to be a good weight.

#define REAQ_MVG_WEIGHT 45.0

//These are the motor speeds alternated between when randomly wandering

#define LOW_SPEED_WANDER 40
#define HI_SPEED_WANDER 30

//Some default values for upper and lower bound colors, but this is dynamically set on startup.
//The lower bound which determines what reading we consider to be black
int COLOR_LOWER_BOUND = 5;
//The upper bound color above which we consider our reading to be white again.
//Actual working values for white are reading around 20-30
int COLOR_UPPER_BOUND = 60;

//These globals track the average for color and distance.
//These values are modified when accessing the get*Avg() functions
float avgColorLeft = 0,
    avgColorRight = 0;
int sonarAvg = 255;


//Helper function for a really terse way to set both motor speeds
void mv(int left, int right) {
    setMotorSpeed(leftMotor, left);
    setMotorSpeed(rightMotor, right);
}

//Helper function for performing a left turn, in place
void turnLeft() {
    long turnTime = 450;
    mv(-55, 55);
    delay(turnTime);
}

//Helper function for right turn left turn.
void turnRight() {
    long turnTime = 450;
    mv(55, -55);
    delay(turnTime);
}

//Let the robot turn in a random direction
void randomDir() {
    long turnProb = 0;
    turnProb = rand() % 100;
    if (turnProb < 50) {
        turnLeft();
    } else {
        turnRight();
    }
}

//Get a moving average with the last average taken (lastAvg),
//the alpha weight desired and the newest reading.
float getMovingAvg(float lastAvg, float alpha, int newestReading) {
    return lastAvg + (alpha * (newestReading - lastAvg));
}

//Sets the moving average for sonar
//In the body is an unused technique to discard outliers by weighting them inversely to
//how far away from the average they are.
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

//Set the color average with the given weight
//Using only the green and blue channel values because red returns with less dynamic range and
//drags the whole average down.
void getColorAvg(float weight) {
    long r, g, b;
    getColorRGB(colorLeft, r, g, b);
    float cLeft = (g + b) / 2.0;

    getColorRGB(colorRight, r, g, b);
    float cRight = (g + b) / 2.0;

    avgColorLeft = getMovingAvg(avgColorLeft, weight / 100.0, cLeft);
    avgColorRight = getMovingAvg(avgColorRight, weight / 100.0, cRight);
    writeDebugStreamLine("left: %f, right: %f", avgColorLeft, avgColorRight);
}

//this is for wandering right
void randomRight() {
    mv(HI_SPEED_WANDER, LOW_SPEED_WANDER);
}

//this is for wandering left
void randomLeft() {
    mv(LOW_SPEED_WANDER, HI_SPEED_WANDER);
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
    while (true) {
        //generates number between 0 and 99.
        //starts with 50/50 chance of going left or right.
        //lower end of probability will always be for turning left.
        turnProb = rand() % 100;
        rTime = rand() % 900 + 300;
        getColorAvg(LOWER_MOVING_AVG_WEIGHT);

        //random turning algorithm.
        if (turnProb < leftProbability) {
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

//Simple predicate, are we off the line?
bool offline(float wt) {
    writeDebugStreamLine("offline");
    getColorAvg(wt);
    //Are both sensors off of the line
    return fabs(avgColorLeft - COLOR_LOWER_BOUND) > 3 && fabs(avgColorRight - COLOR_LOWER_BOUND) > 3;
}

//Play the sound when it's offline
//It's a silly upward tone
void playTheOfflineSound() {
    playTone(100, 25);
    playTone(300, 25);
    playTone(500, 25);
    playTone(700, 25);
}

//Attempt reacquire the line.
// First, skid steer left. If a sensor acquires black, return true
//second turn right for double the amount
bool acquireLine() {
    bool acquired = false;
    mv(-20, 20);

    //Pull a turn leftward until we are no longer offline
    //Turn duration is 1.5 seconds
    for (int i = 0; i < 15 && !acquired; i++) {
        //rotate left about 100
        acquired = acquired || !offline(REAQ_MVG_WEIGHT);
        delay(100);
    }

    mv(20, -20);
    //Turn right for double the amount until we are no longer offline
    for (int i = 0; i < 30 && !acquired; i++) {
        //then rotate right 200
        acquired = acquired || !offline(REAQ_MVG_WEIGHT);

        delay(100);
    }

    if (!acquired) {
        //If we haven't yet acquired the line, play the sound,
        //return the robot back to center from where it began turning
        playTheOfflineSound();
        mv(-20, 20);
        delay(1200);
    } else {
        //Otherwise we have acquired the line, try moving forward for
        //a moment and straight from the direction we acquired the line
        //This will help us leave the fat lines
        mv(10, 10);
        for (int i = 0; i < 15; i++) {
            delay(100);
            getColorAvg(LOWER_MOVING_AVG_WEIGHT);
        }

        //If the end of this maneuver puts us off the line
        //we are probably off the line
        if (offline(LOWER_MOVING_AVG_WEIGHT)) {
            acquired = false;
            playTheOfflineSound();
            //play the not acquired sound
        }
    }

    return acquired;
}

//Set the motor speed based on the observed difference in light readings.
//Turns left when the right sensor high
//Turns right when the left sensor is high
void setMotorSpeedBasedOnLightSensor() {
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
task lineFollow() {
    while (true) {
        getColorAvg(LOWER_MOVING_AVG_WEIGHT);
        //If one of the sensors dips below the threshold, begin attempting to follow a line
        if (fabs(avgColorLeft - COLOR_LOWER_BOUND) < 3 || fabs(avgColorRight - COLOR_LOWER_BOUND) < 3) {
            stopTask(randomWalk);
            setLEDColor(ledRedFlash);

            int offlineCount = 0;

            while (true) {
                //Start loop to follow line
                setMotorSpeedBasedOnLightSensor();
                delay(100);
                getColorAvg(MOVING_AVG_WEIGHT);

                if (offline(MOVING_AVG_WEIGHT)) {
                    setLEDColor(ledOrangeFlash);
                    offlineCount += 1;
                } else {
                    offlineCount = 0;
                }
                //Track how many time we read offline. If we are offline for .5 sec
                //Try to reacquire the line
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

//Do the calibration for the black values
void calBlack() {
    for (int i = 0; i < 10; i++) {
        getColorAvg(MOVING_AVG_WEIGHT);
        delay(100);
    }

    COLOR_LOWER_BOUND = ((avgColorLeft + avgColorRight) / 2);
}

//Do calibration for white values
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
task objectDetect() {
    //stopTask(lineFollow);
    bool detected;
    while (true) {

        if (detected) {//When the object is detected, set a higher weight to the mvg avg
            getSonarAvg(50.0);
        } else {
            delay(100);
            getSonarAvg(15.0);
        }

        //If we detect an object, speed off towards it
        if (sonarAvg < 90 && sonarAvg > STOP_DIST) {
            detected = true;
            stopTask(randomWalk);
            stopTask(lineFollow);
            setLEDColor(ledOff);

            float actualSpeed = (sonarAvg / 90.0) * 128;
            writeDebugStreamLine("speed: %f", actualSpeed);
            mv(actualSpeed, actualSpeed);
            delay(10);
        } else if (sonarAvg <= STOP_DIST && detected) {//Otherwise if we're lower than the stop dist after detection
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
        } else if (sonarAvg > 90 && detected) {//If the object disappears after being detcted
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

task main() {
    //Calibrate black after two seconds on startup
    setLEDColor(ledOrangeFlash);
    delay(2000);
    calBlack();

    //After that, calibrate white
    setLEDColor(ledGreenFlash);
    delay(2000);
    calWhite();

    //Then begin tasks gracefully, spacing them out a bit to avoid weird behavior
    startTask(randomWalk);
    delay(100);
    startTask(lineFollow);
    delay(100);
    startTask(objectDetect);

    while (true);//Never exit main task please.
}
