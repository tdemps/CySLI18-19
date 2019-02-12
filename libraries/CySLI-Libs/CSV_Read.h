//CSV specific libraries:
#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SysCall.h>

File csv;
char delim = ',';
SdFatSdioEX SD;
  
void CSVsetup(){

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
  
  //Burnout      //False when motor is on, true afterwards //output
  
  UpdateData();
  UpdateData(); //these calibrate the prev values for filtering
  UpdateData();
  
  while(accRefine < 30){ // simple launch detection using vertical acc, 100% necessary for test flight/competition
    UpdateData();
    //WriteData();  // uncomment if data before launch is needed
  }

  Serial.println("launch");
  velocity = velPrev = 0; //gets rid of garbage velocity values from sitting on the pad
  delay(2000);
  
  while(!burnout){  //waits until burnout is complete
    UpdateData();
    WriteData();
    if(accRefine <= -(g-3.0) && altRefine > 1500) //pelican //checks if vertical acc is <= ~ -30 && is over a set height
      burnout = true;
  }
  
  LogWrite(7);
  Serial.println(F("leaving Burnout"));
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
      delay(40);
    }
    //altCorrect += 1.5 * (pos-35) / 2;
    while(true){
        UpdateData();
        WriteData();
        altCorrect += 1.2 * (pos-35) / 2;
        if( maxHeight > (altRefine+25) ){ //checks for falling rocket, then closes brakes
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


void csvUpdateData(){
  
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

/*
   Read a file one field at a time.
   file - File to read.
   str - Character array for the field.
   size - Size of str array.
   delim - csv delimiter.
   return - negative value for failure.
            delimiter, '\n' or zero(EOF) for success.
*/
int csvReadText(File* file, char* str, size_t size, char delim) {
  char ch;
  int rtn;
  size_t n = 0;
  while (true) {
    // check for EOF
    if (!file->available()) {
      rtn = 0;
      break;
    }
    if (file->read(&ch, 1) != 1) {
      // read error
      rtn = -1;
      break;
    }
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    if (ch == delim || ch == '\n') {
      rtn = ch;
      break;
    }
    if ((n + 1) >= size) {
      // string too long
      rtn = -2;
      n--;
      break;
    }
    str[n++] = ch;
  }
  str[n] = '\0';
  return rtn;
}
//------------------------------------------------------------------------------
int csvReadInt32(File* file, int32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtol(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadInt16(File* file, int16_t* num, char delim) {
  int32_t tmp;
  int rtn = csvReadInt32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp < INT_MIN || tmp > INT_MAX) return -5;
  *num = tmp;
  return rtn;
}
//------------------------------------------------------------------------------
int csvReadUint32(File* file, uint32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtoul(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadUint16(File* file, uint16_t* num, char delim) {
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp > UINT_MAX) return -5;
  *num = tmp;
  return rtn;
}
//------------------------------------------------------------------------------
int csvReadDouble(File* file, double* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtod(buf, &ptr);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadFloat(File* file, float* num, char delim) {
  double tmp;
  int rtn = csvReadDouble(file, &tmp, delim);
  if (rtn < 0)return rtn;
  // could test for too large.
  *num = tmp;
  return rtn;
}
//------------------------------------------------------------------------------
void csvSetup() {
  // Remove existing file.
//    SD.remove("READTEST.TXT");

  // Write test data.   (Disabled for my test with real data)
//    file.print(F(
//  "36,23.20,20.70,57.60,79.50,01:08:14,23.06.16\r\n"
//  "37,23.21,20.71,57.61,79.51,02:08:14,23.07.16\r\n"
//  ));

//  // Must be dim 9 to allow for zero byte.

  while (csv.available()) {
  if (csvReadUint32(&csv, &time, delim) != delim
        || csvReadDouble(&csv, &altitude, delim) != delim
        || csvReadDouble(&csv, &accX, delim) != delim
        || csvReadDouble(&csv, &accY, delim) != delim
        || csvReadDouble(&csv, &accZ, delim) != '\n'){
    //    || csvReadText(&file, dateS, sizeof(dateS), delim) != '\n') {
      Serial.println("read error");
      int ch;
      delay(100);
      int nr = 0;
      // print part of file after error.
      while ((ch = csv.read()) > 0 && nr++ < 100) {
        delay(10);
        Serial.write(ch);
      }
      break;
    }
    Serial.print(time);
    Serial.print(delim);
    Serial.print(altitude);
    Serial.print(delim);
    Serial.print(accX);
    Serial.print(delim);
    Serial.print(accY);
    Serial.print(delim);
    Serial.println(accZ);
  }
}
