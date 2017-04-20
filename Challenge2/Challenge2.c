#pragma config(Sensor, S1,     colorLeft,         sensorEV3_Color)
#pragma config(Sensor, S3,     colorRight,         sensorEV3_Color)
#pragma config(Sensor, S2,     sonar,         sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          leftWheel,     tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorD,          rightWheel,    tmotorEV3_Large, PIDControl, encoder)



void testSonar () {
	int val = getDistanceValue(sonar);
	writeDebugStreamLine("sonar val = %d", val);
}

void testColor () {
	setColorMode(colorLeft, colorTypeGrayscale_Reflected);
	setColorMode(colorRight, colorTypeGrayscale_Reflected);

	int val = getColorGrayscale(colorLeft),
	    val2 = getColorGrayscale(colorRight);
	writeDebugStreamLine("color left is %d, and color right is %d", val, val2);
}

task main()
{
	while (true) {
		testSonar();
		//testColor();
	}


}
