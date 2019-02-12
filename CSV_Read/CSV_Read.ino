//CSV specific libraries:
#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SysCall.h>
#include <sensors.h>

// CSV FILES MUST BE FORMATTED: TIME,ALTITUDE,AX,AY,AZ

#define g 32.174    // gravity, ft/s^2
#define CDCLOSED 0.7  //The coefficient of drag when the air brakes are closed 
#define AREACLOSED 0.25  //The area of the rocket when the air brakes are closed, ft^2
#define FINALHEIGHT 5280.0  //Final height we want rocket to reach at apogee, in ft
#define AIRDENSITY 0.0023 //Air density of launch field, lb/(g*ft^3)
#define MASS 3   //Mass of the rocket, lb/g
#define BURNOUTHEIGHT 300

bool burnout = false; //current status of motor (false = motor active)
bool brake = false; //status of the brakes (false = closing, true = opening)

//These are not all on the same line to comment them
double AccPrev = 0;      // previous x acceleration
double PnextAcc = 0;     // prediction of next x acceleration for kalman filter, used exclusively in the kalman function
double PnextAlt = 0;    // prediction of next altitude for kalman filter, used exclusively in the kalman function
double PnextVel = 0;    // prediction of next velocity for kalman filter
double altPrev = 0;     // saved altitude from previous loop, ft
double velPrev = 0;
double altRefine = 0;   // smoothed altitude, ft
double accRefine = 0;          // smoothed vertical acceleration, ft/s^2
double velIntegral = 0; // integrated velocity from vertical acceleration, ft/s
double velDerive = 0;   // derivated velocity from altitude, ft/s
double velocity = 0;    // averaged velocity, ft/s
double maxHeight = 0;   // current max height the rocket has reached in current flight, ft
double projHeight = 0;
double altitude = 0.0;
double aCorrect = 0;
double altCorrect = 0;

unsigned long time=0;     // current time, ms
unsigned long OldTime=0;  // time from previous loop, ms


File csv;
char delim = ',';
SdFatSdioEX SD;

//MPU 6050 Accelerations
double accX, accY, accZ;   // unfiltered accelerations, ft/s^2

void setup(){
  SerialSetup();
  BnoBmpSetup();
  ServoSetup();
  SDcardSetup();
  SDcardWriteSetup();
  csv = SD.open("46lb_csv_read.csv", FILE_READ);
  if (!csv) {
    Serial.println("open failed");
    return;
  }
  // Rewind the file for read.
  csv.seek(0);
  
  //csvSetup();
  csv.seek(0);
  delay(1000);
  Serial.println("");   
  
  Burnout();
}

void loop() { // run code (main code) 
  while(csv.available()){
  UpdateData();

  if(burnout){
    ApogeePrediction(velocity);
  }

  EndGame();
  
  WriteData();
  }
  csv.close();
  return;
}
//--------------------------------------------------------------Beginning of the Functions---------------------------------------------

//Serial Setup
void SerialSetup(){
 Serial.begin(9600);
}


//------------------------------------------------------------IMPORTANT FUNCTIONS-------------------------------------------------------
double Kalman(double UnFV,double FR1,double *Pold){

/* function filtering results in real time with Kalman filter */

  char A=1,un=0,H=1,B=0;
  double Prediction,P,y,S,K,FR2;
  float Q = 0.1, R = 0.2;

  // Q=0.1, R=0.2      // defined these from the Matlab code
  // inputs are Unfiltered Response, Filtered  response, Prediction value old
  // make sure to predefine P and possibly other values at the beginning of the main code. starts at 1

  //UnFV    // unfiltered value from sensors read in (zn)           -internal logic
  //FR1     // read this variable in from main (AX(i)) defined last iteration -internal logicexc
  //Pold    // output Pold and read in Pold from last iteration       -input,output
  //Q       // sensor specific value, we might default this           -input
  //R       // sensor specific value, we might default this           -input
  //A         
  //FR2
  
  Prediction= A*FR1 +B*un;    // State Prediction     (Predict where we're goning to be)
  P=A*(*Pold)*A +Q;           // Covariance Prediction  (Predict how much error)
  y=UnFV-H*Prediction;        // Innovation       (Compare reality against prediction)
  S= H*P*H +R;                // innovation Covariance  (Compare real error against prediction)
  K=(P*H)/S;                  // Kalman Gain         (Moderate the prediction)
  FR2=Prediction+K*y;         // state update      (New estimate of where we are)
  *Pold=(1-K*H)*P;            // Covariance update     (New estimate of error)
  
  return FR2;  
}


double ApogeePrediction(double vel){

  /*Predicts apopgee from current velocity, deploys brakes
    if needed. CAN ONLY BE USED AFTER BURNOUT  */
  //all heights are in ft
  //double vel    //current filtered velocity
  //double projHeight;        //The projected final height with no air brakes at this moment in time
  //double k ;           //Drag to be used to calculate projHeight

  //Calcualtes projectedHeight
  double k = 0.5 * AREACLOSED * CDCLOSED * AIRDENSITY; //from old code, check constants
  
  projHeight = (MASS / (2.0 * k)) * log(((MASS * g) +(k * vel * vel)) / (MASS * g)) + altRefine;
  //If projectedHeight will surpass desiredFinalHeight
  
  if (projHeight > (FINALHEIGHT + 30) ) //error of 30ft, pelican
  {
    brake = true;
   // altCorrect += 1.5 * (pos-35) / 2;
  }
  else
  {
    brake = false;
  }
  LogWrite(ServoFunction(brake));    //opens or closes brakes
  return projHeight;
}

double Integrate(unsigned long prevTime, unsigned long currTime, double val) {

  /*function requires two times, and one data point
  DESIGNED FOR ACCELERATION INEGRATION ONLY*/
  
  return val*((currTime-prevTime)/1000.0); //computes and returns Area, the result of the integration
} 
  
double Derive(unsigned long OldTime, unsigned long time, double altPrev, double altRefine){
  
  return (altRefine-altPrev)/( ((double) time -(double) OldTime) / 1000);
}

void Burnout(){

  /* Detects when the vertical acceleration is negative. Holds arduino hostage until it's satisfied*/
  UpdateData();
  UpdateData(); //these calibrate the prev values for filtering
  
  while(accRefine < 5 && altRefine < 30){ // simple launch detection using vertical acc, 100% necessary for real flights
    UpdateData();
   // WriteData();  // uncomment if data before launch is needed
  }
  
  velocity = 0; //gets rid of garbage velocity values from sitting on the pad
  
  while(!burnout){  //waits until burnout is complete
    UpdateData();
    WriteData();
    if(accRefine <= 0 || altRefine > BURNOUTHEIGHT) //pelican //checks if vertical acc is <= ~ -30 && is over a set height
      burnout = true;
  }
  
  LogWrite(7);  //writes burnout event to datalog
  Serial.println(F("Leaving Burnout, starting static braking sequence"));
  delay(500);
  unsigned long burnoutTime = millis();
  brake = true;
  
  while(millis() < (burnoutTime + 8000)){
    LogWrite(ServoFunction(brake));
    UpdateData();
    WriteData();
  }
  brake = false;
  LogWrite(ServoFunction(brake));
  Serial.println("End of static brake deployment");
  delay(1000);
}

void EndGame(){
  
  /* Checks if apogee has been reached or if the rocket is on a poor trajectory. 
  If so, the brakes will permanently close and data will be logged until end of flight*/

  if(  (maxHeight > (altRefine+30) ) && (velocity < -50)){ //checks for falling rocket using altitude and velocity
      Serial.println("Falling, closing brakes");
      LogWrite(3);    
      brake = false;
      ServoFunction(brake); //closes brakes since we set brake to false
      WriteData();
    
      while(true){        //brakes closed, flight data will be logged until computer is turned off
        UpdateData();
        WriteData();
      }
    
  } else if(altRefine > 5280){ // failsafe if rocket goes over 1 mile 
    Serial.println("Too high, brakes open");
    LogWrite(6);
    brake = true;
    while(pos < MAX_ANGLE){   // fully deploys brakes
      ServoFunction(brake);
      WriteData();
    }
    //altCorrect += 1.5 * (pos-35) / 2;
    while(true){
        UpdateData();
        WriteData();
        altCorrect += 1.2 * (pos-35) / 2;
        if( maxHeight > (altRefine+25) ){ //checks for falling rocket, then closes brakes
            Serial.println("Falling");
            LogWrite(3);    
            brake = false;
            ServoFunction(brake); 
    
            while(true){        //brakes closed, flight data will be logged until computer is turned off
                UpdateData();
                WriteData();
            }
       }
    }
  }  
}


void UpdateData(){
  
  /* Updates all global variables for calculations */ 
  
  if (csvReadUint32(&csv, &time, delim) != delim
        || csvReadDouble(&csv, &altitude, delim) != delim
        || csvReadDouble(&csv, &accX, delim) != delim
        || csvReadDouble(&csv, &accY, delim) != delim
        || csvReadDouble(&csv, &accZ, delim) != '\n'){
      Serial.println("update error");
      int ch;
      OldTime = time;
      int nr = 0;
      // print part of file after error.
    //  while ((ch = csv.read()) > 0 && nr++ < 1) {
      //  Serial.write(ch);
      //}
      delay(500);
      return;
    }
    altitude -= altCorrect;
  altRefine = Kalman(altitude, altPrev, &PnextAlt);   //DO NOT TOUCH
  accRefine = Kalman( (accY-aCorrect) , AccPrev, &PnextAcc);       //Only change first parameter for desired vertical acc direction
 // Serial.print(time); Serial.print("  "); Serial.println(OldTime);
  velocity += Integrate(OldTime, time, accRefine); //Integrating to get new velocity  
  //velocity = Kalman( velocity, velPrev, &PnextVel);
  
  //ServoTest();
  SerialTest();
  
  OldTime=time;         // reassigns time for next integration cycle, DO NOT TOUCH
  AccPrev=accRefine;    // reassigns accRefine for initial acceleration at next integration cycle and kalman, DO NOT TOUCH
  altPrev=altRefine;    // reassigns altRefine for initial altitude at next derivation and kalman, DO NOT TOUCH
  velPrev = velocity;   //for kalman filter, DO NOT TOUCH
    
  if(altRefine > maxHeight)
    maxHeight = altRefine;
}

void SerialTest(){

 /*For debugging, uncomment desired variables and call within UpdateData()*/
  /*Serial.print("time: "); */Serial.print(time); Serial.print(" ,");
 // /*Serial.print("rawAlt: "); */Serial.print(altitude); Serial.print(" ,");
  /*Serial.print("altRef: "); */Serial.print(altRefine); Serial.print(" ,");
 // /*Serial.print("aX: "); */Serial.print(accX); Serial.print(" ,");
  /*Serial.print("aY: ");*/ Serial.print(accY); Serial.print(" ,");
  ///*Serial.print("aZ: "); */Serial.print(accZ); Serial.print(" ,");
  /*Serial.print("accRef: "); */Serial.print(accRefine); Serial.print(" ,");
  /*Serial.print("vel: "); */ Serial.print(velocity); Serial.print(" ,");
  Serial.print(getPos()); Serial.print(" ,");
  /*Serial.print("proHeight: "); */ Serial.print(projHeight); Serial.print(" , ");
  Serial.print(altCorrect); Serial.print(" , "); Serial.print(maxHeight);
  Serial.println(""); // prints new line
  delay(100);         // optional delay of output
 
}
