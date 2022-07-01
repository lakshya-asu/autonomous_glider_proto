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


#define PI 3.14159265

#define BNO055_SAMPLERATE_DELAY_MS (50)     // Set pause between samples
enum state{live, idle, glide, helix, land, kill}

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
void 2dcorrection(){                                                                                              // Updated by Manas
double theta_rad=(atan2(curr_y,curr_x))- (atan2(target_long-gps_long,target_lat-gps_lat));  
double theta = theta_rad * 180 / PI;
return theta;
}
};                                         //object
 
 
class control: public plane
{
void select_state();                          //servo output fns
void control();
};

boolean debug = true;                       // true/false = extra/no information over serial

int leftServoPin  = 9;                           // Digital pin for left servo
int rightServoPin= 10;                          // Digital pin for right servo
int elevatorServoPin =11;                       //Digital pin for elevator servo

float roll, pitch, yaw;                     // Variable to hold roll, pitch, yaw information

Adafruit_BNO055 bno = Adafruit_BNO055();    // Use object bno to hold information

Servo leftServo;                            // Create servo rollServo
Servo rightServo;                          // Create servo pitchServo

void select_state()
{
 if(live)
  STATE=idle;
 else if(killswitch)
  STATE=kill;
 else if(theta<=65 && h>10)
  STATE=GLIDE;
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
void turn(){

  int leftOutput = (theta > 0)? 100: 80;          //+servo offset
  int rightOutput =(theta > 0)? 80: 100;          //sign depends on direction of servo //check after mounting
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
//Timer for GPS data
uint32_t timer = millis();                                                                                              // Updated by Manas
//Timer for GPS data
void setup(void) {

  leftServo.attach(leftServoPin);                // The rollServo is connected at rollPin
  rightServo.attach(rightServoPin);              // The pitchServo is connected at pitchPin




GPS.begin(9600);                                                                                              // Updated by Manas
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
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
}

//---- Main Program Loop ----//

void loop(){
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

  leftServo.write(servoRoll);              // Send mapped value to rollServo
  rightServo.write(servoPitch);            // Send mapped value to rollServo
  
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
    distance = sqrt((target_long-gps_long)*(target_long-gps_long)+(target_lat-gps_lat)*(target_lat-gps_lat));
}
