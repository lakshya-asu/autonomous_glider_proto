#include <ServoTimer2.h>
#include <Adafruit_GPS.h> //this is the GPS library from Adafruit
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2); //makes Pin two and three serial data
ports RX (Pin2) and TX (Pin3) on the Arduino Vin (5v) GND(GND)
Adafruit_GPS GPS(&mySerial); //creates the GPS object

String NMEA1; //variable for NMEA sentence one
String NMEA2; //variable for NMEA sentence two
char C; //variable to read characters comming from GPS
float GPSLat; //global Variable to store the GPS Lattitude
float GPSLong; //global Variable to store the GPS Longitude
const float GPSDESTLAT = 41.740882; //constant value for the latitude of the center of the quad 28
const float GPSDESTLONG = -111.812801; //constant value for the longitude of the center of the quad

#define max 1850
#define min 1250
#define change 1

ServoTimer2 horizontal; // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0; // variable to store the servo position

void setup() {
 horizontal.attach(5); // attaches the servo on pin 9 to the servo object
 Serial.begin(57600);
 //GPS setup stuff
 Serial.println("GPS setup");
 //GPS.sendCommand("$PMTK251,57600*2C");
//set baud rate to 57600
 GPS.begin(9600);
//Turns on GPS at 9600 baud
 GPS.sendCommand("$PGCMD,33,0*6D");
//Turns the antenna update data off (we don't care if we have an antena)
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//Sets the update break to one hert frequency (10,5,or 1)
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//Tells the GPS to only send the RMC and the GGA NMEA sentances
}

void loop() {
 //Local variable declarations:
 Serial.println("starting here");
 float heading = 180;
 float homeHeading = 0;
 while(1)
 {
 horizontalTurn(heading, homeHeading);
 heading = updateHeading(heading);
 homeHeading = updateHeading(500);
 
 Serial.print("heading: ");
 Serial.println(heading);
 Serial.print("home Heading: ");
 Serial.println(homeHeading);
 }
}
//-----------code that controlls the servos--------------
void horizontalTurn(float currentHeading, float headingHome)
{
 double magnitude = 0;
 int goLeft;
 int degLeft;
 int degRight;
 int debug = 1; //set to one to display data and add a delay
 int turn = 0;
 int turnMultiplier = 1.25;
 if(currentHeading < headingHome)
 {
 degLeft = currentHeading + (360 - headingHome);
 }
 else
 {
 degLeft = currentHeading - headingHome;
 }
 degRight = 360 - degLeft;
 if (degLeft > degRight) //right hand turn
 {
 Serial.println("turning right");
 magnitude = (degRight / 180.0);
 turn = 90 - magnitude * (40);
turn = (turn - 50) * (50 / 3) + 800;
 }
 else //left Hand turn
 {
 Serial.println("turning left");
 magnitude = degLeft / 180.0;
 turn = 90 + magnitude * (60); // number between 0 and 60.
 turn = turn * turnMultiplier;
turn = (turn - 90) * (40 / 3) + 1500;
 }
 if (debug)
 {
 Serial.print("deg for left turn: ");
 Serial.println(degLeft);
 Serial.print("deg for right turn: ");
 Serial.println(degRight);
 Serial.print("the mag is ");
 Serial.println(magnitude);
 Serial.print("the turn is ");
 Serial.println(turn);
 Serial.println();
 delay(500);
 }
 horizontal.write(turn);
 Serial.print("turn write: ");
 Serial.println(turn);
}
//------------code that updates heading (LOTS of trig here)----------
int updateHeading(int oldHeading)
{
 int heading;
 float oldLat; //saves old lattitude
 float oldLong; //set to one if you want to Debug
 int dontUpdate;
 float staticLat = GPSLat;
 float staticLong = GPSLong;
 if (oldHeading == 500)
 {
 readGPS();
 oldLat = GPSLat;
oldLong = GPSLong;
 GPSLat = GPSDESTLAT;
 GPSLong = GPSDESTLONG;
 dontUpdate = 1;
 }
 else {
 heading = oldHeading;
 oldLat = GPSLat;
 oldLong = GPSLong;
 readGPS(); //updates current lattitude and longitude
 Serial.println("here");
 dontUpdate = 0;
 }
 float distance = 0.000038; //.000001 = 3.149606299212598 inches
 if (oldLat > (GPSLat + distance) || oldLat < (GPSLat - distance) || oldLong >(GPSLong + distance) || oldLong < (GPSLong - distance))//Compares old position and new position to see if the glider has moved.
 {
 //End of Debugging crap
 double deltaY = GPSLat - oldLat; //finds the distance moved East to West
 double deltaX = GPSLong - oldLong; //finds the distance moved North to South *GPS longitude INCREASES as you go south
 if (deltaY == 0 || deltaX == 0) { Serial.println("hey you idiot this is equal to a zero you are going to die!"); }
 if (deltaX > 0 && deltaY > 0) //quadrant one
 {
 heading = (90 - atan(deltaY / deltaX)*(180 / 3.14159265358797)); //Calculates the new angle N=0 deg, E = 90 deg, S = 180 deg, W = 270 deg
 }
 else if (deltaY > 0 && deltaX < 0) //quadrant two
 {
 heading = (270 - atan(deltaY / deltaX)*(180 / 3.14159265358797)); //calculates the new angle N=0 deg, E = 90 deg, S = 180 deg, W = 270 deg
 }
 else if (deltaY < 0 && deltaX < 0) //quadrant three {
 heading = (270 - atan(deltaY / deltaX)*(180 / 3.14159265358797)); //calculates the new angle N=0 deg, E = 90 deg, S = 180 deg, W = 270 deg
 }
 else if (deltaY < 0 && deltaX > 0) //quadrant four
 {
 heading = (90 - atan(deltaY / deltaX)*(180 / 3.14159265358797)); //calculates the new angle N=0 deg, E = 90 deg, S = 180 deg, W = 270 deg
 }
 else if (deltaY == 0 && deltaX > 0) { heading = 90; } //this line catches if deltaY is zero and changes the heading accordingly.
 else if (deltaY == 0 && deltaX < 0) { heading = 270; } //this line catches if deltaY is zero and changes the heading accordingly.
 else if (deltaY > 0 && deltaX == 0) { heading = 0; } //this line catches if deltaX is zero and changes the heading accordingly.
 else if (deltaY < 0 && deltaX == 0) { heading = 180; } //this line catches if deltaX is zero and changes the heading accordingly.
 if (!dontUpdate)
 {
 Serial.println("New Heading <------------------------------------------------");
 }
 oldHeading = heading; //returns new heading
 }
 else
 {
 GPSLat = oldLat; //if the glider didn't move enough then
 GPSLong = oldLong;
 }
 if (dontUpdate) // if were were just calculating the heading needed to go home then don't change where we are at
 {
 GPSLat = staticLat; //sets current lat to what it was
 GPSLong = staticLong; //sets current Long to what it was
 }
 return oldHeading; //since the glider didn't move the heading doesn't need to change.
}
void readGPS()
{
 int debug = 1;
 clearGPS();
  while (!GPS.newNMEAreceived()) //loops until we get a NEMA sentence
 {
 C = GPS.read();
 }
 GPS.parse(GPS.lastNMEA()); //parses the last NMEA sentence
 NMEA1 = GPS.lastNMEA(); //save the sentence into NMEA1
 //don't do anything here or you will miss the second sentence
 while (!GPS.newNMEAreceived()) //loops until we get a NEMA sentence
 {
 C = GPS.read(); //here we start the second GPS read
 }
 GPS.parse(GPS.lastNMEA()); //parses the last NMEA sentence
 NMEA2 = GPS.lastNMEA(); //save the sentence into NMEA2
 //convert lattitude degrees into degrees, minutes, and seconds
 int GPSLatDeg = GPS.latitude / 100; //this gets the degrees value
 float GPSLatMinDeg = GPS.latitude - (GPSLatDeg * 100); //this gets the minutes and seconds part by itself (still in degrees)
 float GPSLatMinSec = (GPSLatMinDeg / 60); //this converts the minutes and seconds value from decimals into their actual values
 GPSLat = GPSLatDeg + GPSLatMinSec; //this adds the decimals to the minutes and seconds
 //convert latitude degrees into degrees, minutes, and seconds
 int GPSLongDeg = GPS.longitude / 100; //this gets the degrees value
 float GPSLongMinDeg = GPS.longitude - (GPSLongDeg * 100); //this gets the minutes and seconds part by itself (still in degrees)
 float GPSLongMinSec = (GPSLongMinDeg / 60); //this converts the minutes and seconds value from decimals into their actual values
 GPSLong = -1 * (GPSLongDeg + GPSLongMinSec); //this adds the decimals to the minutes and seconds
 if (debug)
 {
 Serial.println(NMEA1); //prints first NEMA sentence
 Serial.println(NMEA2); //prints second NEMA sentence
 }
}
//clears the corrupt data from GPS by reading twice and throwing away data
void clearGPS()
{
 for (int i = 0; i < 3; i++)
 {
 while (!GPS.newNMEAreceived()) //loops until we get a NEMA sentence
 {
 C = GPS.read();
 }
 }
}
