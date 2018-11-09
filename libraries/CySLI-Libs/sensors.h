

/**
	This file contains functions related to sensors connected to Teensy.
	All setup parameters and data fetching (ex. getAccel, getAlt) should
	be maintained here. All functions should be commented and have a summary of their
	use, inputs, and outputs.
	
	**/


#ifndef sensors_h	//DONT TOUCH
#define sensors_h	//ALSO DONT TOUCH


//include all sensor libraries below.

//ex. include <BMP280.h>

#include <i2c_t3.h>   // provides simple and intuitive interfaces to I2C devices
//#include <Wire.h>   // allows you to communicate with I2C / TWI devices
#include <Servo.h>    // servo library
#include <SPI.h>    // Serial Peripheral Interface(SPI) used for communicating with one or more peripheral devices quickly over short distances
#include <limits.h>
#include <BNOBMPLib.h>

//set output pins for various sensors below
#define SERVO_PIN 47	//servo signal output pin.
#define INIT_ANGLE 35	//servo's inital angle, or the "closed" angle for the brakes
#define MAX_ANGLE 125	//maximum angle that brakes can be deployed
#define SD_PIN BUILTIN_SDCARD		//data pin for SD Card

//C++ objects representing devices attached to Teensy/Arduinos.
Servo servo;        // airbrake servo
double baseline = 0.0;         // baseline pressure, taken during IMU setup


short pos = INIT_ANGLE;

/**
  * This function is an example to show documentation style.
  * Inputs: None
  * Returns: int of constant 1.
**/
int testingLib(){
	return 1;
}

//-----------------------------------------ACCELEROMETER-FUNCTIONS--------------------------------

void BnoBmpSetup(){
	DebugSetup();

}

void GetAcc(double *accX, double *accY, double *accZ){
	
	int16_t arr[3];
	readAccelData(arr);
	
	*accX =  (double) ( (double) arr[0] / 1000 * 32); //converting from mg to ft/s^2
	*accY = (double) ( (double) arr[1] / 1000 * 32);
	*accZ = (double) ( (double) arr[2] / 1000 * 32);
	
    // Now we'll calculate the accleration value into actual mg's
    //accX = accX; // - accelBias[0];  // subtract off calculated accel bias
    //accY = accY; // - accelBias[1];
    //accZ = accZ; // - accelBias[2]; 
	
}



//-------------------------------------------ALTIMETER-FUNCTIONS--------------------------------//



double GetAlt(){

  
  if(baseline == 0.0){
	  Serial.print("Calibrating Barometer...");
	  for(int i = 0; i < 20; i++){
		baseline += BMPAltitude(); 
		delay(500);
	  }
	 baseline = baseline / 20;
	 Serial.println("Calibration Complete");
  }
  
  return BMPAltitude() - baseline; //should be in feet
  }


//------------------------------------------------SERVO-FUNCTIONS-------------------------------------//
short ServoFunction(bool brake){
  
  /*Function that will close and open brakes. If brake a true, 
    we want brakes to open. If false, brakes will closed. */
	
  if(brake){
      if(pos < MAX_ANGLE){
		pos += 5;
        servo.write(pos);	  // opens brakes 5 more degrees
		return 4;
      }
  } else if (pos > INIT_ANGLE){
      pos = INIT_ANGLE;     //closes brakes 
      servo.write(pos);
	  return 5;
  }
  return -1;
}

void ServoSetup(){
  servo.attach(SERVO_PIN);
  servo.write(INIT_ANGLE);
}

short getPos(){
	return pos;
}

#endif