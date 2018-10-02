


/* 
 Demonstrates basic BNO055 functionality including parameterizing the register addresses, 
 initializing the sensor, communicating with pressure sensor BMP280, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 
 
 Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms. 
 Can compare results to hardware 9 DoF sensor fusion carried out on the BNO055.
 
 This sketch is intended specifically for the BNO055+BMP280 Add-On Shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 
 The Add-on shield can also be used as a stand-alone breakout board for any Arduino, Teensy, or 
 other microcontroller by closing the solder jumpers on the back of the board.
  
 The BMP280 is a simple but high resolution (20-bit) pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 All sensors communicate via I2C at 400 Hz or higher.
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the BNO055_BMP280 breakout board.
 
 Hardware setup:
 Breakout Board --------- Arduino/Teensy
 3V3 ---------------------- 3.3V
 SDA -----------------------A4/17
 SCL -----------------------A5/16
 GND ---------------------- GND
 
 Note: The BNO055_BMP280 breakout board is an I2C sensor and uses the Arduino Wire or Teensy i2c_t3.h library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 The Teensy has no internal pullups and we are using the Wire.begin function of the i2c_t3.h library
 to select 400 Hz i2c speed.
 */
//#include <Wire.h>   
#include <i2c_t3.h>
#include <SPI.h>
#include<BNOBMPLib.h>

// Using the BNO055_BMP280 breakout board/Teensy 3.1 Add-On Shield, ADO is set to 1 by default 
#define ADO 1
#if ADO
#define BNO055_ADDRESS 0x29   //  Device address of BNO055 when ADO = 1
#define BMP280_ADDRESS 0x77   // Address of BMP280
#else
#define BNO055_ADDRESS 0x28   //  Device address of BNO055 when ADO = 0
#define BMP280_ADDRESS 0x77   // Address of BMP280
#endif  

// Set initial input parameters
enum Ascale {  // ACC Full Scale
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_18G
};

enum Abw { // ACC Bandwidth
  ABW_7_81Hz = 0,
  ABW_15_63Hz,
  ABW_31_25Hz,
  ABW_62_5Hz,
  ABW_125Hz,    
  ABW_250Hz,
  ABW_500Hz,     
  ABW_1000Hz,    //0x07
};

enum APwrMode { // ACC Pwr Mode
  NormalA = 0,  
  SuspendA,
  LowPower1A,
  StandbyA,        
  LowPower2A,
  DeepSuspendA
};

enum Gscale {  // gyro full scale
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS    // 0x04
};

enum GPwrMode { // GYR Pwr Mode
  NormalG = 0,
  FastPowerUpG,
  DeepSuspendedG,
  SuspendG,
  AdvancedPowerSaveG
};

enum Gbw { // gyro bandwidth
  GBW_523Hz = 0,
  GBW_230Hz,
  GBW_116Hz,
  GBW_47Hz,
  GBW_23Hz,
  GBW_12Hz,
  GBW_64Hz,
  GBW_32Hz
};

enum OPRMode {  // BNO-55 operation modes
  CONFIGMODE = 0x00,
// Sensor Mode
  ACCONLY,
  MAGONLY,
  GYROONLY,
  ACCMAG,
  ACCGYRO,
  MAGGYRO,
  AMG,            // 0x07
// Fusion Mode
  IMU,
  COMPASS,
  M4G,
  NDOF_FMC_OFF,
  NDOF            // 0x0C
};

enum PWRMode {
  Normalpwr = 0,   
  Lowpower,       
  Suspendpwr       
};

enum Modr {         // magnetometer output data rate  
  MODR_2Hz = 0,     
  MODR_6Hz,
  MODR_8Hz,
  MODR_10Hz,  
  MODR_15Hz,
  MODR_20Hz,
  MODR_25Hz, 
  MODR_30Hz 
};

enum MOpMode { // MAG Op Mode
  LowPower = 0,
  Regular,
  EnhancedRegular,
  HighAccuracy
};

enum MPwrMode { // MAG power mode
  Normal = 0,   
  Sleep,     
  Suspend,
  ForceMode  
};

enum Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum Mode {
  BMP280Sleep = 0,
  forced,
  forced2,
  normal
};

enum SBy {
  t_00_5ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_2000ms,
  t_4000ms,
};

// Specify BMP280 configuration
uint8_t Posr = P_OSR_16, Tosr = T_OSR_02, Mode = normal, IIRFilter = BW0_042ODR, SBy = t_62_5ms;     // set pressure amd temperature output data rate
// t_fine carries fine temperature as global value for BMP280
int32_t t_fine;
//
uint8_t GPwrMode = NormalG;    // Gyro power mode
uint8_t Gscale = GFS_250DPS;  // Gyro full scale
//uint8_t Godr = GODR_250Hz;    // Gyro sample rate
uint8_t Gbw = GBW_23Hz;       // Gyro bandwidth
//
uint8_t Ascale = AFS_2G;      // Accel full scale
//uint8_t Aodr = AODR_250Hz;    // Accel sample rate
uint8_t APwrMode = NormalA;    // Accel power mode
uint8_t Abw = ABW_31_25Hz;    // Accel bandwidth, accel sample rate divided by ABW_divx
//
//uint8_t Mscale = MFS_4Gauss;  // Select magnetometer full-scale resolution
uint8_t MOpMode = Regular;    // Select magnetometer perfomance mode
uint8_t MPwrMode = Normal;    // Select magnetometer power mode
uint8_t Modr = MODR_10Hz;     // Select magnetometer ODR when in BNO055 bypass mode

uint8_t PWRMode = Normalpwr;    // Select BNO055 power mode
uint8_t OPRMode = NDOF;       // specify operation mode for sensors
uint8_t status;               // BNO055 data status register
float aRes, gRes, mRes;       // scale resolutions per LSB for the sensors
  
// Pin definitions
int intPin = 8;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13;

uint16_t Pcal[8];         // calibration constants from BMP280 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw BMP280 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
int16_t quatCount[4];   // Stores the 16-bit signed quaternion output
int16_t EulCount[3];    // Stores the 16-bit signed Euler angle output
int16_t LIACount[3];    // Stores the 16-bit signed linear acceleration output
int16_t GRVCount[3];    // Stores the 16-bit signed gravity vector output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempGCount, tempMCount;      // temperature raw count output of mag and gyro
float   Gtemperature, Mtemperature;  // Stores the BNO055 gyro and LIS3MDL mag internal chip temperatures in degrees Celsius
double Temperature, Pressure;        // stores BMP280 pressures sensor pressure and temperature
int32_t rawPress, rawTemp;   // pressure and temperature raw count output for BMP280

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float Pitch, Yaw, Roll;
float LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

// BMP280 compensation parameters
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

void DebugSetup()
{
//  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  // Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  // delay(4000);
  // Serial.begin(38400);
  
  // // Set up the interrupt pin, its set as active high, push-pull
  // pinMode(intPin, INPUT);
  // pinMode(myLed, OUTPUT);
  // digitalWrite(myLed, HIGH);
  
  I2Cscan(); // check for I2C devices on the bus8
  
  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("BNO055 9-axis motion sensor...");
  byte c = readByte(BNO055_ADDRESS, BNO055_CHIP_ID);  // Read WHO_AM_I register for BNO055
  Serial.print("BNO055 Address = 0x"); Serial.println(BNO055_ADDRESS, HEX);
  Serial.print("BNO055 WHO_AM_I = 0x"); Serial.println(BNO055_CHIP_ID, HEX);
  Serial.print("BNO055 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.println(" I should be 0xA0");  
  delay(1000); 
  
   
    // Read the WHO_AM_I register of the accelerometer, this is a good test of communication
  byte d = readByte(BNO055_ADDRESS, BNO055_ACC_ID);  // Read WHO_AM_I register for accelerometer
  Serial.print("BNO055 ACC "); Serial.print("I AM "); Serial.print(d, HEX); Serial.println(" I should be 0xFB");  
  delay(1000); 
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte e = readByte(BNO055_ADDRESS, BNO055_MAG_ID);  // Read WHO_AM_I register for magnetometer
  Serial.print("BNO055 MAG "); Serial.print("I AM "); Serial.print(e, HEX); Serial.println(" I should be 0x32");

  delay(1000);   
  
  // Read the WHO_AM_I register of the gyroscope, this is a good test of communication
  byte f = readByte(BNO055_ADDRESS, BNO055_GYRO_ID);  // Read WHO_AM_I register for LIS3MDL
  Serial.print("BNO055 GYRO "); Serial.print("I AM "); Serial.print(f, HEX); Serial.println(" I should be 0x0F");
  delay(1000); 

  if (c == 0xA0) // BNO055 WHO_AM_I should always be 0xA0
  {  
    Serial.println("BNO055 is online...");
    
    // Check software revision ID
    byte swlsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_LSB);
    byte swmsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_MSB);
    Serial.print("BNO055 SW Revision ID: "); Serial.print(swmsb, HEX); Serial.print("."); Serial.println(swlsb, HEX); 
    Serial.println("Should be 03.04");
    
    // Check bootloader version
    byte blid = readByte(BNO055_ADDRESS, BNO055_BL_REV_ID);
    Serial.print("BNO055 bootloader Version: "); Serial.println(blid); 
    
    // Check self-test results
    byte selftest = readByte(BNO055_ADDRESS, BNO055_ST_RESULT);
    
    if(selftest & 0x01) {
      Serial.println("accelerometer passed selftest"); 
    } else {
      Serial.println("accelerometer failed selftest"); 
    }
    if(selftest & 0x02) {
      Serial.println("magnetometer passed selftest"); 
    } else {
      Serial.println("magnetometer failed selftest"); 
    }  
    if(selftest & 0x04) {
      Serial.println("gyroscope passed selftest"); 
    } else {
      Serial.println("gyroscope failed selftest"); 
    }      
    if(selftest & 0x08) {
      Serial.println("MCU passed selftest"); 
    } else {
      Serial.println("MCU failed selftest"); 
    }
      
    delay(1000);
  
  // Read the WHO_AM_I register of the BMP280 this is a good test of communication
  byte f = readByte(BMP280_ADDRESS, BMP280_ID);  // Read WHO_AM_I register for BMP280
  Serial.print("BMP280 "); 
  Serial.print("I AM "); 
  Serial.print(f, HEX); 
  Serial.print(" I should be "); 
  Serial.println(0x58, HEX);
  Serial.println(" ");
  delay(1000); 

  writeByte(BMP280_ADDRESS, BMP280_RESET, 0xB6); // reset BMP280 before initilization
  delay(100);

  BMP280Init(); // Initialize BMP280 altimeter
  Serial.println("Calibration coeficients:");
  Serial.print("dig_T1 ="); 
  Serial.println(dig_T1);
  Serial.print("dig_T2 ="); 
  Serial.println(dig_T2);
  Serial.print("dig_T3 ="); 
  Serial.println(dig_T3);
  Serial.print("dig_P1 ="); 
  Serial.println(dig_P1);
  Serial.print("dig_P2 ="); 
  Serial.println(dig_P2);
  Serial.print("dig_P3 ="); 
  Serial.println(dig_P3);
  Serial.print("dig_P4 ="); 
  Serial.println(dig_P4);
  Serial.print("dig_P5 ="); 
  Serial.println(dig_P5);
  Serial.print("dig_P6 ="); 
  Serial.println(dig_P6);
  Serial.print("dig_P7 ="); 
  Serial.println(dig_P7);
  Serial.print("dig_P8 ="); 
  Serial.println(dig_P8);
  Serial.print("dig_P9 ="); 
  Serial.println(dig_P9);
 
  accelgyroCalBNO055(accelBias, gyroBias);
  
  Serial.println("Average accelerometer bias (mg) = "); Serial.println(accelBias[0]); Serial.println(accelBias[1]); Serial.println(accelBias[2]);
  Serial.println("Average gyro bias (dps) = "); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
  
  delay(1000); 
  
  magCalBNO055(magBias);
  
  Serial.println("Average magnetometer bias (mG) = "); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
  
  delay(1000); 
  
  // Check calibration status of the sensors
  uint8_t calstat = readByte(BNO055_ADDRESS, BNO055_CALIB_STAT);
  Serial.println("Not calibrated = 0, fully calibrated = 3");
  Serial.print("System calibration status "); Serial.println( (0xC0 & calstat) >> 6);
  Serial.print("Gyro   calibration status "); Serial.println( (0x30 & calstat) >> 4);
  Serial.print("Accel  calibration status "); Serial.println( (0x0C & calstat) >> 2);
  Serial.print("Mag    calibration status "); Serial.println( (0x03 & calstat) >> 0);
  
  initBNO055(); // Initialize the BNO055
  Serial.println("BNO055 initialized for sensor mode...."); // Initialize BNO055 for sensor read 
 
  }
  else
  {
    Serial.print("Could not connect to BNO055: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}


void notloop()
{  

    // readGyroData(gyroCount);  // Read the x/y/z adc values
    // // Calculate the gyro value into actual degrees per second
    // gx = (float)gyroCount[0]/16.; // - gyroBias[0];  // subtract off calculated gyro bias
    // gy = (float)gyroCount[1]/16.; // - gyroBias[1];  
    // gz = (float)gyroCount[2]/16.; // - gyroBias[2];   

    // readMagData(magCount);  // Read the x/y/z adc values   
    // // Calculate the magnetometer values in milliGauss
    // mx = (float)magCount[0]/1.6; // - magBias[0];  // get actual magnetometer value in mGauss 
    // my = (float)magCount[1]/1.6; // - magBias[1];  
    // mz = (float)magCount[2]/1.6; // - magBias[2];   
    
    // readQuatData(quatCount);  // Read the x/y/z adc values   
    // // Calculate the quaternion values  
    // quat[0] = (float)(quatCount[0])/16384.;    
    // quat[1] = (float)(quatCount[1])/16384.;  
    // quat[2] = (float)(quatCount[2])/16384.;   
    // quat[3] = (float)(quatCount[3])/16384.;   
    
    readEulData(EulCount);  // Read the x/y/z adc values   
    // Calculate the Euler angles values in degrees
    Yaw = (float)EulCount[0]/16.;  
    Roll = (float)EulCount[1]/16.;  
    Pitch = (float)EulCount[2]/16.;   
 
    readLIAData(LIACount);  // Read the x/y/z adc values   
    // Calculate the linear acceleration (sans gravity) values in mg
    LIAx = (float)LIACount[0];  
    LIAy = (float)LIACount[1];  
    LIAz = (float)LIACount[2];   

    readGRVData(GRVCount);  // Read the x/y/z adc values   
    // Calculate the linear acceleration (sans gravity) values in mg
    GRVx = (float)GRVCount[0];  
    GRVy = (float)GRVCount[1];  
    GRVz = (float)GRVCount[2];   
    
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  
  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
   
       // check BNO-055 error status at 2 Hz rate
    uint8_t sysstat = readByte(BNO055_ADDRESS, BNO055_SYS_STATUS); // check system status
    Serial.print("System Status = 0x"); Serial.println(sysstat, HEX);
    if(sysstat == 0x05) Serial.println("Sensor fusion algorithm running");
    if(sysstat == 0x06) Serial.println("Sensor fusion not algorithm running");
    
    if(sysstat == 0x01) {
       uint8_t syserr = readByte(BNO055_ADDRESS, BNO055_SYS_ERR);
      if(syserr == 0x01) Serial.println("Peripheral initialization error");
      if(syserr == 0x02) Serial.println("System initialization error");
      if(syserr == 0x03) Serial.println("Self test result failed");
      if(syserr == 0x04) Serial.println("Register map value out of range");
      if(syserr == 0x05) Serial.println("Register map address out of range");
      if(syserr == 0x06) Serial.println("Register map write error");
      if(syserr == 0x07) Serial.println("BNO low power mode no available for selected operation mode");
      if(syserr == 0x08) Serial.println("Accelerometer power mode not available");
      if(syserr == 0x09) Serial.println("Fusion algorithm configuration error");
      if(syserr == 0x0A) Serial.println("Sensor configuration error");    
    }  

    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)ax);  
    Serial.print(" ay = "); Serial.print((int)ay); 
    Serial.print(" az = "); Serial.print((int)az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mx ); 
    Serial.print(" my = "); Serial.print( (int)my ); 
    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
    
    Serial.print("qx = "); Serial.print(q[0]);
    Serial.print(" qy = "); Serial.print(q[1]); 
    Serial.print(" qz = "); Serial.print(q[2]); 
    Serial.print(" qw = "); Serial.println(q[3]); 
    Serial.print("quatw = "); Serial.print(quat[0]);
    Serial.print(" quatx = "); Serial.print(quat[1]); 
    Serial.print(" quaty = "); Serial.print(quat[2]); 
    Serial.print(" quatz = "); Serial.println(quat[3]); 
    } 
    
    tempGCount = readGyroTempData();  // Read the gyro adc values
    Gtemperature = (float) tempGCount; // Gyro chip temperature in degrees Centigrade
   // Print gyro die temperature in degrees Centigrade      
    Serial.print("Gyro temperature is ");  Serial.print(Gtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of a degree C
 
   

    // if(SerialDebug) {
      // Serial.println("BMP280:");
      // Serial.print("Altimeter temperature = "); 
      // Serial.print( Temperature, 2); 
      // Serial.println(" C"); // temperature in degrees Celsius
      // Serial.print("Altimeter temperature = "); 
      // Serial.print(9.*Temperature/5. + 32., 2); 
      // Serial.println(" F"); // temperature in degrees Fahrenheit
      // Serial.print("Altimeter pressure = "); 
      // Serial.print(Pressure, 2);  
      // Serial.println(" mbar");// pressure in millibar
      // Serial.print("Altitude = "); 
      // Serial.print(altitude, 2); 
      // Serial.println(" feet");
      // Serial.println(" ");
    // }
   
 /*  
  
    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
    
  
*/
    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }


//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void readAccelData(int16_t *accX, int16_t *accY, int16_t *accZ)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  *accX = ((int16_t)rawData[1] << 8) | rawData[0] ;      // Turn the MSB and LSB into a signed 16-bit value
  *accY = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  *accZ = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

int8_t readGyroTempData()
{
  return readByte(BNO055_ADDRESS, BNO055_TEMP);  // Read the two raw data registers sequentially into data array 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readQuatData(int16_t * destination)
{
  uint8_t rawData[8];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_QUA_DATA_W_LSB, 8, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;
}

void readEulData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_EUL_HEADING_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readLIAData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readGRVData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_GRV_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void initBNO055() {
   // Select BNO055 config mode
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
   delay(25);
   // Select page 1 to configure sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x01);
   // Configure ACC
   writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | Ascale );
   // Configure GYR
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale );
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, GPwrMode);
   // Configure MAG
   writeByte(BNO055_ADDRESS, BNO055_MAG_CONFIG, MPwrMode << 5 | MOpMode << 3 | Modr );
   
   // Select page 0 to read sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);

   // Select BNO055 gyro temperature source 
   writeByte(BNO055_ADDRESS, BNO055_TEMP_SOURCE, 0x01 );

   // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
   writeByte(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01 );
   
   // Select BNO055 system power mode
   writeByte(BNO055_ADDRESS, BNO055_PWR_MODE, PWRMode );
 
   // Select BNO055 system operation mode
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );
   delay(25);
  }

void accelgyroCalBNO055(float * dest1, float * dest2) 
{
  uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
 
  Serial.println("Accel/Gyro Calibration: Put device on a level surface and keep motionless! Wait......");
  delay(4000);
  
  // Select page 0 to read sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
   // Select BNO055 system operation mode as AMG for calibration
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
   delay(25);
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG);
   
 // In NDF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz, set it the same here
   writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | AFS_4G );
   sample_count = 256;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    accel_bias[0]  += (int32_t) accel_temp[0];
    accel_bias[1]  += (int32_t) accel_temp[1];
    accel_bias[2]  += (int32_t) accel_temp[2];
    delay(20);  // at 62.5 Hz ODR, new accel data is available every 16 ms
   }
    accel_bias[0]  /= (int32_t) sample_count;  // get average accel bias in mg
    accel_bias[1]  /= (int32_t) sample_count;
    accel_bias[2]  /= (int32_t) sample_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) 1000;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) 1000;}

    dest1[0] = (float) accel_bias[0];  // save accel biases in mg for use in main program
    dest1[1] = (float) accel_bias[1];  // accel data is 1 LSB/mg
    dest1[2] = (float) accel_bias[2];          

 // In NDF fusion mode, gyro full scale is at +/- 2000 dps, ODR is 32 Hz
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, Gbw << 3 | GFS_2000DPS );
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, GPwrMode);for(ii = 0; ii < sample_count; ii++) {
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;  // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
    delay(35);  // at 32 Hz ODR, new gyro data available every 31 ms
   }
    gyro_bias[0]  /= (int32_t) sample_count;  // get average gyro bias in counts
    gyro_bias[1]  /= (int32_t) sample_count;
    gyro_bias[2]  /= (int32_t) sample_count;
 
    dest2[0] = (float) gyro_bias[0]/16.;  // save gyro biases in dps for use in main program
    dest2[1] = (float) gyro_bias[1]/16.;  // gyro data is 16 LSB/dps
    dest2[2] = (float) gyro_bias[2]/16.;          

  // Return to config mode to write accelerometer biases in offset register
  // This offset register is only used while in fusion mode when accelerometer full-scale is +/- 4g
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
  delay(25);
  
  //write biases to accelerometer offset registers ad 16 LSB/dps
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB, (int16_t)accel_bias[0] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB, ((int16_t)accel_bias[0] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB, (int16_t)accel_bias[1] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB, ((int16_t)accel_bias[1] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB, (int16_t)accel_bias[2] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB, ((int16_t)accel_bias[2] >> 8) & 0xFF);
  
  // Check that offsets were properly written to offset registers
//  Serial.println("Average accelerometer bias = "); 
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB))); 
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB))); 
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB)));

   //write biases to gyro offset registers
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB, (int16_t)gyro_bias[0] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB, ((int16_t)gyro_bias[0] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB, (int16_t)gyro_bias[1] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB, ((int16_t)gyro_bias[1] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB, (int16_t)gyro_bias[2] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB, ((int16_t)gyro_bias[2] >> 8) & 0xFF);
  
  // Select BNO055 system operation mode
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );

 // Check that offsets were properly written to offset registers
//  Serial.println("Average gyro bias = "); 
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB))); 
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB))); 
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB)));

   Serial.println("Accel/Gyro Calibration done!");
}

void magCalBNO055(float * dest1) 
{
  uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};
 
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
  // Select page 0 to read sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
   // Select BNO055 system operation mode as NDOF for calibration
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
   delay(25);
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG );

 // In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
   sample_count = 256;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if (ii == 0) {
        mag_max[jj] = mag_temp[jj]; // Offsets may be large enough that mag_temp[i] may not be bipolar! 
        mag_min[jj] = mag_temp[jj]; // This prevents max or min being pinned to 0 if the values are unipolar...
      } else {
        if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
        if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
      }
    }
    delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
   }

 //   Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
 //   Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
 //   Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0] / 1.6;  // save mag biases in mG for use in main program
    dest1[1] = (float) mag_bias[1] / 1.6;  // mag data is 1.6 LSB/mg
    dest1[2] = (float) mag_bias[2] / 1.6;          

  // Return to config mode to write mag biases in offset register
  // This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
  delay(25);
  
  //write biases to accelerometer offset registers as 16 LSB/microTesla
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB, (int16_t)mag_bias[0] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB, (int16_t)mag_bias[1] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB, (int16_t)mag_bias[2] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB, ((int16_t)mag_bias[2] >> 8) & 0xFF);
 
  // Check that offsets were properly written to offset registers
//  Serial.println("Average magnetometer bias = "); 
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB))); 
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB))); 
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB)));
  // Select BNO055 system operation mode
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );
  delay(25);
  
   Serial.println("Mag Calibration done!");
}


int32_t readBMP280Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BMP280_ADDRESS, BMP280_TEMP_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

int32_t readBMP280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BMP280_ADDRESS, BMP280_PRESS_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

void BMP280Init()
{
  // Configure the BMP280
  // Set T and P oversampling rates and sensor mode
  writeByte(BMP280_ADDRESS, BMP280_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
  // Set standby time interval in normal mode and bandwidth
  writeByte(BMP280_ADDRESS, BMP280_CONFIG, SBy << 5 | IIRFilter << 2);
  // Read and store calibration data
  uint8_t calib[24];
  readBytes(BMP280_ADDRESS, BMP280_CALIB00, 24, &calib[0]);
  dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t bmp280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}

// I2C scan function

void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}

// I2C read/write functions for the BNO055 sensor

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
double BMPAltitude(){
	 rawPress =  readBMP280Pressure();
    Pressure = (float) bmp280_compensate_P(rawPress)/25600.; // Pressure in mbar
    rawTemp =   readBMP280Temperature();
    Temperature = (float) bmp280_compensate_T(rawTemp)/100.;

    return (double) 145366.45f*(1.0f - pow((Pressure/1013.25f), 0.190284f));
}
