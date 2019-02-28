#include <math.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <NewPing.h>

// BT Parsing Variables
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
char messageFromPC[numChars];
boolean newData = false;

// BT Variables 
#define HC06 Serial1
char data = 0;
char points;
float currentLat = 0;
float currentLong = 0;
float targetLat = 0.0;
float targetLong = 0.0;


// Calculation Variables
int distanceToTarget;
int OriginalDistanceToTarget;
int targetHeading;
int currentHeading;
int ErrorDistance = 3;
int angleError = 10;

// GPS Variables
HardwareSerial mySerial = Serial2;
Adafruit_GPS GPS(&Serial2);
#define GPSECHO false // set false to turn off echoing data to serial console, true to debug and see raw GPS sentences
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Motor Control Variables
int motorx = 4; //motor on pin 3 (IN4)
int motory = 5; //motor on pin 4 (IN3)
int motorw = 7; // motorfront on pin 8 (IN2)front
int motorz = 9; //motor front on pin 9 (IN1)front
int modulation1 = 8; //enB
int velocidad; //motor speed
int steeringPot = A1;

// Directions for motor control
#define header_Tolerance = 2
#define LEFT 'l'
#define RIGHT 'r'
#define FORWARD 'f'
#define BACK 'b'
#define STOP 'x'

// Sonar Variables
const int TRIGGER_PIN = 12;  
const int ECHO_PIN = 11;  
#define MAX_DISTANCE 250
#define Alert_Distance 100
#define Halt_Distance 30

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Moving Average Variables
const int ReadingAmt = 4;
int readings[ReadingAmt];
int readI;
int total;
int average;
int cm;

// Test Variables
Servo myservo1;
Servo myservo2;
int pos1 = 0;
int pos2 = 0;

void setup()
{
  for(int intReading = 0; intReading < ReadingAmt; intReading++){
      readings[intReading] = 0;
  }
  pinMode(19, INPUT);
  digitalWrite(19, HIGH);
  pinMode(13, OUTPUT);
  
  pinMode(motorx, OUTPUT);
  digitalWrite(motorx,LOW);
  pinMode(motory, OUTPUT);
  digitalWrite(motory,LOW);
  pinMode(motorw, OUTPUT);
  digitalWrite(motorw,LOW);
  pinMode(motorz, OUTPUT);
  digitalWrite(motorz,LOW);
  
  Serial.begin(9600);
  HC06.begin(9600);
  BTCheck();
  myservo1.attach(11);
  myservo2.attach(10);
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC (recommended minimum) and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);  // Request updates on antenna status, comment out to keep quiet
  useInterrupt(true);
  delay(1000);
}

void loop()
{
 getGPSFix();
 getAngle();
 SendCoordinatesBT();
 recvWithStartEndMarkers();
 if (newData == true) {
     strcpy(tempChars, receivedChars);
     parseData();
     showParsedData();
     newData = false;
 }

 if ( targetLat == 0){
     allOff();
 }
 else {
 
    distanceToWaypoint();
    Serial.print("Distance to target: ");
    Serial.println(distanceToTarget);
    headingAngle(); 
    Serial.println("Target Heading: ");
    Serial.println(targetHeading);
    if(distanceToTarget > ErrorDistance){
//     SonarScan();
     movingAverage();
     calcTurn();
       }
     else{
      allOff();
     }
 }
}

//***************Bluetooth Functions****************
void BTCheck(){
   while(data != '1')
  {
    data = HC06.read();
    data == '0';
   // Serial.write(data);
    //HC06.write(data);
    if(data == '1'){             // Checks whether value of data is equal to 1
         digitalWrite(13, HIGH);   //If value is 1 then LED turns ON
    }
  }
}

void SendCoordinatesBT(){
 HC06.print(currentLat,5);
 HC06.print("|");
 HC06.print(currentLong,5);
 HC06.print("|");
 delay(1000);
} // sendCoordinatesBT

//***************Bluetooth Parsing Functions****************

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (HC06.available() > 0 && newData == false) {
        rc = HC06.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
} // recvWithStartEndMarkers

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ",");
    targetLat = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    targetLong = atof(strtokIndx);     // convert this part to a float
} // parseData

void showParsedData() {
    Serial.print("Latitude: ");
    Serial.println(targetLat, 5);
    Serial.print("Longitude: ");
    Serial.println(targetLong, 5);
} // showParsedData

//***************Calculation Functions****************

int distanceToWaypoint() 
{
  
  float num = radians(currentLong - targetLong);
  float sdlong = sin(num);
  float cdlong = cos(num);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  num = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  num = sq(num); 
  num += sq(clat2 * sdlong); 
  num = sqrt(num); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  num = atan2(num, denom); 
  distanceToTarget =  num * 6372795; 
   
  // check to see if we have reached the current waypoint
  //  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
  //    nextWaypoint();
    
  return distanceToTarget;
}  // distanceToWaypoint()

int headingAngle() 
{
  float difflon = radians(targetLong-currentLong);
  float aLat = radians(currentLat);
  float bLat = radians(targetLat);
  float x = sin(difflon) * cos(bLat);
  float y = sin(aLat) * cos(bLat) * cos(difflon);
  y = cos(aLat) * sin(bLat) - y;
  float theta = atan2(x, y);
  if (theta < 0.0)
  {
    theta += TWO_PI;
  }
  targetHeading = degrees(theta);
  return targetHeading;
}   // courseToWaypoint()

//***************GPS Functions****************

SIGNAL(TIMER0_COMPA_vect){ // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  // writing direct to UDR0 is much much faster than Serial.print but only one character can be written at a time.
#endif
} // SIGNAL(TIMER0_COMPA_vect)

void useInterrupt(boolean v) {
  if(v){ // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
  else{ // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
} // useInterrupt()

uint32_t timer = millis();
// used for GPS to work

void getGPSFix(){
  if(! usingInterrupt){ // in case you are not using the interrupt above, you'll need to 'hand query' the GPS, not suggested :(
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if(GPSECHO)
      if(c) Serial.print(c);
  }
  if (GPS.newNMEAreceived()) { // if a sentence is received, we can check the checksum, parse it...
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  if (timer > millis())  timer = millis();  // if millis() or timer wraps around, we'll just reset it
  if (millis() - timer > 1000) {  // approximately every 1 second or so, print out the current stats
    timer = millis(); // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 5);
      currentLat = GPS.latitudeDegrees, 5;
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees, 5);
      currentLong = GPS.longitudeDegrees, 5;
      Serial.println();
    }
  }
} // getGPSFix()

void getAngle(){
  currentHeading = GPS.angle;
 // Serial.print("Tracking Angle (Compass Direction): ");
 // Serial.println(GPS.angle); 
} // getAngle()

//***************Motor Functions****************

void calcTurn(){
    int heading_Error = targetHeading - currentHeading;

   //if( average >= Alert_Distance){ 

     if( heading_Error <= angleError){
        forward(130);
        delay(5000);
        allOff();
        delay(5000);
        
      }
      else if( targetHeading > 0 && targetHeading <180){
       // left(70);
        delay(100);
        forward(130);
        delay(5000);
        allOff();
        delay(5000);
      }
      else if( targetHeading > 180 && targetHeading < 360){
      //  right(70);
        delay(100);
        forward(130);
        delay(5000);
        allOff();
        delay(5000);
      }
  // }

//   if( average < Alert_Distance > Halt_Distance){
//    
//      if( heading_Error <= angleError){
//        forward(60);
//      }
//      else if( targetHeading > 0 && targetHeading <180){
//        left(60);
//        delay(100);
//        forward(60);
//      }
//      else if( targetHeading > 180 && targetHeading < 360){
//        right(60);
//        delay(100);
//        forward(60);
//      }
//   }
//
//   if( average < Halt_Distance){
//     allOff();
//     while( average < Halt_Distance){
//       backward(60);
//     }
//     allOff();
//   }
} // calcTurn()

void allOff() // function to stop motors
  {
    //backside motor
    digitalWrite(motorx, LOW);
    digitalWrite(motory, LOW);
 
    //front side motor 
    digitalWrite(motorw, LOW);
    digitalWrite(motorz, LOW);
    Serial.println("Motors Off");
  } // allOff()

void forward(int Speed){ // function to move forward
    Serial.println("going forward");
    digitalWrite(motorx, HIGH);
    digitalWrite(motory, LOW);
    analogWrite(modulation1, Speed);
} // forward()

void backward(int Speed){ // function to move backward
    digitalWrite(motorx, LOW);
    digitalWrite(motory, HIGH);
    analogWrite(modulation1, Speed);
} // backward()

void left(int Speed){ // function to turn left
  Serial.println("going Left");
  digitalWrite(motorw, LOW);
  digitalWrite(motorz, HIGH);
  //delay(2000);
  analogWrite(modulation1, Speed);
} // left()

void right(int Speed){ // function to turn right
  Serial.println("going Right");
  digitalWrite(motorw, HIGH);
  digitalWrite(motorz, LOW);
 // delay(2000);
  analogWrite(modulation1, Speed);
} // right()

//***************Object Detection Functions****************

int SonarScan(){
  cm = sonar.ping_cm();
 // Serial.println(cm);
}

void movingAverage(){
    total = total - readings[readI];
  readings[readI] = cm;
  total = total + readings[readI];
  readI = readI + 1;
  if(readI >= ReadingAmt){
    readI = 0;
  }
  average = total/ReadingAmt;
}


