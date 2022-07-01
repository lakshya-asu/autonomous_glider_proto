//---- Included Libraries ----//
#include <Wire.h>                           // I²C library
#include <math.h>                           // trig functions
#include <Adafruit_Sensor.h>                // Base library for sensors
#include <Adafruit_BNO055.h>                // BNO055 specific library
#include <utility/imumaths.h>               // Vector, Matrix, and IMUMath library
#include <Servo.h>                        // Standard Servo library
#include <PID_v1.h>                       //PID library
#include <Adafruit_GPS.h>
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t timer = millis();


//boolean debug = true;                       // true/false = extra/no information over serial

int leftServoPin  = 9;                           // Digital pin for left servo
int rightServoPin= 10;                          // Digital pin for right servo
int elevatorServoPin =11;                       //Digital pin for elevator servo

float roll, pitch, yaw;                     // Variable to hold roll, pitch, yaw information

Adafruit_BNO055 bno = Adafruit_BNO055();    // Use object bno to hold information

Servo leftServo;                            // Create servo rollServo
Servo rightServo;                          // Create servo pitchServo

//PID algorithm parametres

//Define Variables we'll be connecting to
double SetpointRoll, InputRoll, OutputRoll, InputRollPrev = 0.0;

//Specify the links and initial tuning parameters
double Kp = 20, Ki = 0, Kd = 0;
PID myPIDRoll(&servoRoll, &OutputRoll, &SetpointRoll, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double SetpointPitch, InputPitch, OutputPitch, InputPitchPrev = 0.0;
PID myPIDPitch(&servoPitch, &OutputPitch, &SetpointPitch, Kp, Ki, Kd, DIRECT);

#define PI 3.14159265
#define SENSE_PIN 13
#define BNO055_SAMPLERATE_DELAY_MS (50)     // Set pause between samples
enum state{live, idle, glide, helix_init, helix, land, kill}

class plane{
public:
float RollAngle, YawAngle, PitchAngle;
float altitude, 2dcorr, positionX, positionY, vcurr;
plane(
str st;

void updater();
}plane;                                          //object

class ideal: public plane
{
public:
float gps_long, gps_lat, distance;
float target_lat = 19.023322;
float target_long = 72.02020110; 
gps_parse();
float distance_to_target =  distanceBetween(gps_lat, gps_long, target_lat, target_long);                                                                                                      // Update on 5-2-21 : 10:10PM
float targetHeading1 =      courseTo(gps_lat,gps_long,target_lat,target_long);                                                                                                      // Update on 5-2-21 : 10:10PM
}
}id;                                         //object
 
 
class control: public plane
{
void select_state();                          //servo output fns
void control();
}ctrl;

void select_state()
{
 if(live)
  STATE=idle;
 else if(killswitch)
  STATE=kill;
 else if(theta<=65 && h>10)
  STATE=GLIDE;
  else if(distance_to_target>25 && distance_to_target<30)
  STATE=HELIX_INIT;
 else if(theta>65 && h>10)
  STATE=HELIX;
 else if(h<10)
  STATE=LAND;
 else STATE=idle;
}

void controller(state STATE)
{
  switch(STATE):
    case glide: glide();
      break;
    case kill: kill();
      break;
    case HELIX_INIT: helix_init();
      break;
    case helix: helix();
      break;
    case land: lander();
      break;
    default: idle();
    break;
}

void glide(){                                                                                              // Updated by Manas
//compute PID for theta
  2dcorrection();
  videal=16;
  if(target_distance > 30)  snake();
  
//PID
//PID(vcurr, theta, videal)
  leftOutput=rightOutput;
  while(theta > 2 || theta < -2) turn();
}

void snake(){                                                                                              // Updated by Manas
  // Go Right
  int leftOutput = 80;          //+servo offset
  int rightOutput =100;          //sign depends on direction of servo //check after mounting
 // WAIT FOR SOME TIME (((((WAIT CODE IS PENDING)))))
// Go Left
  int leftOutput = 100;          //+servo offset
  int rightOutput =80;          //sign depends on direction of servo //check after mounting
}
void turn(){

  int leftOutput = (theta > 0)? 100: 80;          //+servo offset
  int rightOutput =(theta > 0)? 80: 100;          //sign depends on direction of servo //check after mounting
  }
void helix_init(){
float on_left_edge;
gps_parse();
float right_edge_lat,right_edge_long,left_edge_lat,left_edge_long;//Left edge and right edge is known. (30ft away from centre)
float distance_to_right_edge =  distanceBetween(gps_lat, gps_long, right_edge_lat, right_edge_long);
float distance_to_left_edge =  distanceBetween(gps_lat, gps_long, left_edge_lat, left_edge_long);
while(distance_to_left_edge > 2 || distance_to_right_edge > 2) //Go to the left or right edge until distance is around 2 ft. Once reached, Helix left or right.
{  
if(distance_to_left_edge < distance_to_right_edge){
    on_left_edge= 1; //Turn Left
  int leftOutput = 100;          //Turn left
  int rightOutput =80;          
}
else{
    on_left_edge= 0; 
  int leftOutput = 80;          //Turn Right
  int rightOutput =100;          
}
}
}
void helix(){
//gps_parse();
//float right_edge_lat,right_edge_long,left_edge_lat,left_edge_long;//Left edge and right edge is known. (30ft away from centre)
//float distance_to_right_edge =  distanceBetween(gps_lat, gps_long, right_edge_lat, right_edge_long);
//float distance_to_left_edge =  distanceBetween(gps_lat, gps_long, left_edge_lat, left_edge_long);
//if(distance_to_left_edge < distance_to_right_edge){
//  turnleft = 1; //Turn Left
//}
//else{
//  turnleft = 0; //Turn Right
//}
  int leftOutput = (on_left_edge)? 100: 80;          //Condition for Left Helix or Right helix 
  int rightOutput =(!on_left_edge)? 80: 100;          
  }

//Timer for GPS data
uint32_t timer = millis();                                                                                              // Updated by Manas
//Timer for GPS data
void setup(void) {
    int deployed = 0;
pinMode(SENSE_PIN,INPUT);
  leftServo.attach(leftServoPin);                // The rollServo is connected at rollPin
  rightServo.attach(rightServoPin);              // The pitchServo is connected at pitchPin
GPS.begin(9600);                                                                                              // Updated by Manas
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
 GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
 
  Serial.begin(115200);                     // Create serial connection at 115,000 Baud

  if (!bno.begin())                         // Attempt communication with sensor
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  delay(100);                               // Wait 0.1 seconds to allow it to initialize
  bno.setExtCrystalUse(true);               // Tell sensor to use external crystal
   myPIDRoll.SetOutputLimits(-90, 90);
  myPIDPitch.SetOutputLimits(-90, 90);
  myPIDRoll.SetSampleTime(20);
  myPIDPitch.SetSampleTime(20);

  SetpointPitch = 0;
  SetpointRoll = 0;

  myPIDRoll.SetMode(AUTOMATIC);
  myPIDPitch.SetMode(AUTOMATIC);
}

//---- Main Program Loop ----//

void loop(){
  deployed = digitalRead(SENSE_PIN);
  while(!deployed){
    idle();
  }
imu_compute();

}
void imu_compute() {

  //---- Request Euler Angles from Sensor ----//
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
double curr_x = euler.x()                                                                                  // Current X coordinates
double curr_y = euler.y();

  int servoRoll =  map(euler.y(),  -90, 90, -60, 60);
  int servoPitch = map(euler.z(), -180, 180, 60, 60);

  int leftOutput = ((servoRoll + servoPitch) / 2) + 90;   //+servo offset
  int rightOutput = ((servoRoll - servoPitch) / 2) + 90;  //sign depends on direction of servo //check after mounting
myPIDRoll.Compute();
  myPIDPitch.Compute();
  
  leftServo.write(OutputRoll);              // Send mapped value to rollServo
  rightServo.write(OutputPitch);            // Send mapped value to rollServo
  
  delay(BNO055_SAMPLERATE_DELAY_MS);       // Wait before rerunning loop
}


void gps_parse(){
   char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
 if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

   if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    gps_lat = GPS.latitude;
    gps_long = GPS.longitude;
    //distance = sqrt((target_long-gps_long)*(target_long-gps_long)+(target_lat-gps_lat)*(target_lat-gps_lat));
}



double distanceBetween(double lat1, double long1, double lat2, double long2)                                                                                                      // Update on 5-2-21 : 10:10PM
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double courseTo(double lat1, double long1, double lat2, double long2)                                                                                                      // Update on 5-2-21 : 10:10PM
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);                                     //check quadrant
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}
