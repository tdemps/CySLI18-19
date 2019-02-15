 // formatting rules:
// 1) { on same line as statement
//   ex: if (condition) {
// 2) multicomment lines go above line
//  ex: // comment
//      // comment continued
//      variable = 0;
// 3) single line comments on same line, tabbed and spaced
//  ex: variable = 0; // comment
// 5) variables have camel case starting with lowercase
//  ex: helloWorld = 0;
// 6) functions have camel case starting with uppercase
//  ex: HelloWorld()

#include <sensors.h>
#include <SD.h>

// ROCKET CONSTANTS
#define g 32.174    // gravity, ft/s^2
#define CDCLOSED 0.75  //The coefficient of drag when the air brakes are closed 
#define AREACLOSED 0.08  //The frontal area of the rocket when the air brakes are closed, ft^2
#define FINALHEIGHT 4750.0  //Apogee we want rocket to reach, in ft
#define AIRDENSITY 0.0023 //Air density of launch field, lb/(g*ft^3)
#define MASS 3   //Mass of the rocket, lb/g
#define BURNOUTHEIGHT 1200 //minimum height for brake actuation

// Testing parameters
#define SERIALTEST 1
#define SERVOTEST 1


bool burnout = false; //current status of motor (false = motor active)
bool brake = false; //status of the brakes (false = closing, true = opening)

double AccPrev = 0;      // previous vertical acceleration
double PnextAcc = 0;     // prediction of next vertical acceleration for kalman filter, used exclusively in the kalman function
double PnextAlt = 0;     // prediction of next altitude for kalman filter, used exclusively in the kalman function
double PnextVel = 0;     // prediction of next velocity for kalman filter, ft/s
double altPrev = 0;      // saved altitude from previous loop, ft
double velPrev = 0;      // previous velocity of velocity, ft/s
double altRefine = 0;    // filtered altitude, ft
double accRefine = 0;    // filtered vertical acceleration, ft/s^2
double velIntegral = 0;  // integrated velocity from vertical acceleration, ft/s
double velDerive = 0;    // derivated velocity from altitude, ft/s
double velocity = 0;     // averaged velocity, ft/s
double maxHeight = 0;    // current max height the rocket has reached in current flight, ft
double projHeight = 0;   // predicted agpogee, ft
double altitude = 0.0;  //raw altitude taken from ms5611 altimeter, ft

unsigned long time=0;     // current time, ms
unsigned long OldTime=0;  // time from previous loop, ms


//Accelerations
double accX = 0.0, accY = 0.0, accZ = 0.0;   // raw accelerations, ft/s^2

void setup(){
  SerialSetup();
  Serial.print("Hello world");
  const int ledPin = 13;
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);   // set the LED on
  BnoBmpSetup();
  ServoSetup();
  SDcardSetup();    
  Burnout();
  digitalWrite(ledPin, LOW);    // set the LED off
}

void loop() { // run code (main code) 
  
  UpdateData();
  if(SERVOTEST)
    ServoTest();
  if(burnout && SERVOTEST == 0)
    ApogeePrediction(velocity);

  EndGame();
  
  WriteData();
}

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
  //double vel              //current velocity
  //double projHeight;      //The projected final height with no air brakes at this moment in time
  //double k ;              //Drag to be used to calculate projHeight

  //Calcualtes projectedHeight
  double k = 0.5 * AREACLOSED * CDCLOSED * AIRDENSITY; //from old code, check constants
  
  projHeight = (MASS / (2.0 * k)) * log(((MASS * g) +(k * vel * vel)) / (MASS * g)) + altRefine;

  if (projHeight > FINALHEIGHT ) //error of 15ft, pelican
  {
    brake = true;
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
  
  return (altRefine-altPrev)/( ((double) time -(double) OldTime) / 1000.0);
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
    if(accRefine <= 0 && altRefine > BURNOUTHEIGHT) //pelican //checks if vertical acc is <= ~ -30 && is over a set height
      burnout = true;
  }
  
  LogWrite(7);  //writes burnout event to datalog
  Serial.println(F("Leaving Burnout, starting static braking sequence"));
  
  unsigned long burnoutTime = millis();
  brake = true;
  
  while(millis() < (burnoutTime + 6000)){
    LogWrite(ServoFunction(brake));
    UpdateData();
    WriteData();
  }
  brake = false;
  LogWrite(ServoFunction(brake));
  Serial.println("End of static brake deployment");
}

void EndGame(){
  
  /* Checks if apogee has been reached or if the rocket is on a poor trajectory. 
  If so, the brakes will permanently close and data will be logged until end of flight*/

  if(maxHeight > (altRefine+50)){ //checks for falling rocket using altitude and velocity
    FallingProtocol();
    
  } else if(altRefine > FINALHEIGHT){ // failsafe if rocket goes over desired height 
    
    LogWrite(6);
    brake = true;
    
    while(getPos() < MAX_ANGLE){   // fully deploys brakes
      LogWrite(ServoFunction(brake));
      WriteData();
      UpdateData();
    }
    
    while(true){
        UpdateData();
        WriteData();
        if( maxHeight > (altRefine+25) ){ //checks for falling rocket, then closes brakes
           FallingProtocol();
       }
    }
  }  
}

void FallingProtocol(){

  /*Closes brakes and locks program into loop for remainder of flight*/
  
  LogWrite(3);    
  brake = false;
  LogWrite(ServoFunction(brake)); 
  
  while(true){        //flight data will be logged until computer is turned off
      UpdateData();
      WriteData();
  }
  
}
void UpdateData(){
  
  /* Updates all global variables for calculations */
  
  time = millis();
  GetAcc(&accX, &accY, &accZ);
  accX -= 32.1;
  altitude  = GetAlt();

  altRefine = Kalman(altitude, altPrev, &PnextAlt);
  accRefine = Kalman(accX, AccPrev, &PnextAcc);
  
  velocity += Integrate(OldTime, time, accRefine); //Integrating to get new velocity  

  if(SERIALTEST)
    SerialTest();
  
  OldTime=time;         // reassigns time for next integration cycle (move this to top of loop to eliminate any time delays between time and oldTime to have a better integration and derivation?)
  AccPrev=accRefine;    // reassigns accRefine for initial acceleration at next integration cycle and kalman
  altPrev=altRefine;    // reassigns altRefine for initial altitude at next derivation and kalman
  velPrev = velocity;   // saves velocity for next kalman cycle
  delay(10);
  if(altRefine > maxHeight) //keep max height stored for apogee confirmation
    maxHeight = altRefine;
}

void ServoTest(){ 
  
  /*opens and closes brakes constantly for brake/servo check */
  
    Serial.println(getPos());
    if(getPos() == INIT_ANGLE)
      brake = true;
    if(getPos() == MAX_ANGLE){
      brake = false;
      Serial.println("Closing brakes");
    }
  LogWrite(ServoFunction(brake));
  delay(500);

}

void SerialTest(){

 /*For debugging, uncomment desired variables and 
     call this function within UpdateData()*/
     
  Serial.print("Time: "); Serial.print(time); Serial.print(", ");
  Serial.print("rawAlt: "); Serial.print(altitude); Serial.print(", ");
  Serial.print("AltRefine: "); Serial.print(altRefine); Serial.print(", ");
  Serial.print("accX: " ); Serial.print(accX); Serial.print(", ");
  Serial.print("accY: "); Serial.print(accY); Serial.print(", ");
  Serial.print("accZ: " ); Serial.print(accZ); Serial.print(", ");
  Serial.print("AccRefine: "); Serial.print(accRefine); Serial.print(", ");
  Serial.print("Servo Angle: "); Serial.print(getPos()); Serial.print(", ");
  Serial.print("Vel: "); Serial.print(velocity); Serial.print(" ,");
  
  Serial.println(""); // prints new line
  delay(100);         // optional delay of output
 
}
