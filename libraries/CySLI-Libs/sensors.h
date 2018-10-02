

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

#include <I2Cdev.h>   // provides simple and intuitive interfaces to I2C devices
#include <MPU6050.h>  // IMU library
#include <Wire.h>   // allows you to communicate with I2C / TWI devices
#include <Servo.h>    // servo library
#include <SPI.h>    // Serial Peripheral Interface(SPI) used for communicating with one or more peripheral devices quickly over short distances
#include <limits.h>
#include <MS5611.h>

//set output pins for various sensors below
#define SERVO_PIN 47	//servo signal output pin.
#define INIT_ANGLE 35	//servo's inital angle, or the "closed" angle for the brakes
#define MAX_ANGLE 125	//maximum angle that brakes can be deployed
#define SD_PIN 53		//data pin for SD Card

//C++ objects representing devices attached to Teensy/Arduinos.
MS5611 ms5611;      //IMU1
MPU6050 mpu;        //IMU2
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
void MpuSetup(){
    //joins I2C bus (I2Cdev library doesn't do this automatically) (Was found in available rocket code on internet) 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    mpu.setFullScaleAccelRange(3);  //0=2g, 1=4g, 2=8g, 3=16g 
    mpu.setFullScaleGyroRange(2); //0=250 deg/s, 1=500 deg/s, 2=1000 deg/s, 3=2000 deg/s 
}
void BnoBmpSetup(){
	
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
	delay(4000);
	Serial.begin(38400);
  
	// Set up the interrupt pin, its set as active high, push-pull
	pinMode(intPin, INPUT);
	pinMode(myLed, OUTPUT);
	digitalWrite(myLed, HIGH);
 
	//DebugSetup();
  
	writeByte(BMP280_ADDRESS, BMP280_RESET, 0xB6); // reset BMP280 before initilization
	delay(100);
  
	BMP280Init(); // Initialize BMP280 altimeter
    accelgyroCalBNO055(accelBias, gyroBias);
	delay(1000); 
  
	magCalBNO055(magBias);
	delay(1000); 

	initBNO055(); // Initialize the BNO055

}

void GetAcc(int16_t *accX, int16_t *accY, int16_t *accZ){
	
	readAccelData(int16_t *accX, int16_t *accY, int16_t *accZ);
	
    // Now we'll calculate the accleration value into actual mg's
    accX = (float)accX // - accelBias[0];  // subtract off calculated accel bias
    accY = (float)accY; // - accelBias[1];
    accZ = (float)accZ; // - accelBias[2]; 
	
}

// void GetAcc(int16_t *accX, int16_t *accY, int16_t *accZ){
   // mpu.getAcceleration(accX, accY, accZ);
   // *accX = map(*accX, 0, 2048, 0, 32);
   // *accY = map(*accY, 0, 2048, 0, 32)-32; //if y is pointing up (or usb port) subtract 31
   // *accZ = map(*accZ, 0, 2048, 0, 32); //pelican
 
   // /* Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 // * (Register 28). For each full scale setting, the accelerometers' sensitivity
 // * per LSB in ACCEL_xOUT is shown in the table below:
 // *
 // * <pre>
 // * AFS_SEL | Full Scale Range | LSB Sensitivity
 // * --------+------------------+----------------
 // * 0       | +/- 2g           | 8192 LSB/mg
 // * 1       | +/- 4g           | 4096 LSB/mg
 // * 2       | +/- 8g           | 2048 LSB/mg
 // * 3       | +/- 16g          | 1024 LSB/mg */
// }


//-------------------------------------------ALTIMETER-FUNCTIONS--------------------------------//

void ms5611Setup(){
  ms5611.begin();
  baseline = ms5611.readPressure();
}

// double GetPressure(){
  // return ms5611.readPressure();
// }

void Resetms5611(){
	
  baseline = ms5611.readPressure();
  
}

double GetAlt(){

  /*Reads current air pressure to obtain altitude*/
  
double a,P;
  
  // Get a new pressure reading:
  P = ms5611.readPressure();

  // Show the relative altitude difference between
  // the new reading and the baseline reading:
  a = ms5611.getAltitude(P,baseline);
  
  //convert to ft and return
  return a*3.28084; 
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