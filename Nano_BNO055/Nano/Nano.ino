#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include "PID_v1.h"
#include <Servo.h>
#define SENSE_PIN 13
int deployed_flag = 0;
//double gps_lat, gps_long, target_lat, target_long;
double gps_lat = 19.099417;
double gps_long = 72.902356;
double target_lat = 19.100023;
double target_long = 72.902426;
double roll = 0;
double pitch = 0;
double temp = 0;
int videal = 16;
#define OFFSET_PITCH .11
#define ALPHA .75

double prev_roll = -255;
double prev_pitch = -255;

int servoValL = 0;
int servoValR = 0;
//PID stuff
double setpoint_pitch, input_pitch, output_pitch;
double kp_pitch = 2, ki_pitch = 0.1, kd_pitch = 0.25; //tuning values to change here
PID stable_pitch(&input_pitch, &output_pitch, &setpoint_pitch, kp_pitch, ki_pitch, kd_pitch, DIRECT); //pitch pid loop

double setpoint_roll, input_roll, output_roll;
double kp_roll = 8, ki_roll = 0.1, kd_roll = 0.25; //tuning values to change here
PID stable_roll(&input_roll, &output_roll, &setpoint_roll, kp_roll, ki_roll, kd_roll, DIRECT); //roll pid loop

#define BNO055_SAMPLERATE_DELAY_MS (50)
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//servo stuff
Servo servoL;
Servo servoR;

void setup(void)
{
  pinMode(SENSE_PIN, INPUT);
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  servoL.attach(4);
  servoR.attach(5);
  servoL.write(90);
  servoR.write(90);


  input_pitch = 0;
  setpoint_pitch = 0;
  //stable_pitch.SetSampleTime(40);
  stable_pitch.SetOutputLimits(-90, 90);
  stable_pitch.SetSampleTime(50);
  stable_pitch.SetMode(AUTOMATIC);
  stable_roll.SetOutputLimits(-180, 180);
  stable_roll.SetSampleTime(50);
  stable_roll.SetMode(AUTOMATIC);
}

void loop() {
  int deployed = !digitalRead(SENSE_PIN);
  int killswitch = 0; //  RF Logic pending
  int height_from_ground = 20; //  Sensor logic pending
  if (deployed) deployed_flag = 1;
  while (!deployed_flag) {
    digitalWrite(13, HIGH);
    delay(200); //  Fast blinking of built_in led will indicate that the code is functional even when everything's in idle position
    digitalWrite(13, LOW);
    delay(200);
  }

  get_imu_data();
  if (killswitch) {
    servoL.write(120); //  Pitch straight downwards
    servoR.write(120);
  }
  float distance_to_target = distanceBetween(gps_lat, gps_long, target_lat, target_long);
  float targetHeading1 = courseTo(gps_lat, gps_long, target_lat, target_long);
  Serial.println(distance_to_target );
  Serial.println(targetHeading1);

  if (distance_to_target > 10) {
    glide_straight(distance_to_target, targetHeading1); //  Should be in horizontal position while starting the loop
  }
  else if (height_from_ground > 10 && distance_to_target <= 10) {
    helix_init();
  }
  else if (height_from_ground < 10 && distance_to_target <= 10) {
    lander();
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}


void lander() { //  Same as glide function
  temp = zero_pitch(pitch);
  temp = conv_rads_to_degs( temp );
  input_pitch = temp;
  write_pitch( output_pitch );
  Serial.print("output_PID_pitch: ");
  Serial.println(output_pitch);
  temp = conv_rads_to_degs( roll );
  input_roll = temp;
  write_roll( output_roll );
  Serial.print("output_PID_roll: ");
  Serial.println(output_roll);
  servoL.write(servoValL);
  servoR.write(servoValR);
  Serial.print("Servo Data L: ");
  Serial.println(servoValL);
  Serial.print("Servo Data R: ");
  Serial.println(servoValR);

}
void helix_init() {
  float on_left_edge;
  float right_edge_lat, right_edge_long, left_edge_lat, left_edge_long; //Left edge and right edge is known. (30ft away from centre)
  float distance_to_right_edge =  distanceBetween(gps_lat, gps_long, right_edge_lat, right_edge_long);
  float distance_to_left_edge =  distanceBetween(gps_lat, gps_long, left_edge_lat, left_edge_long);
  while (distance_to_left_edge > 2 || distance_to_right_edge > 2) //Go to the left or right edge until distance is around 2 ft. Once reached, Helix left or right.
  {
    if (distance_to_left_edge < distance_to_right_edge) {
      on_left_edge = 1;
      //  turn left
      servoL.write(120);
      servoR.write(60);
    }
    else {
      on_left_edge = 0;
      //  turn Right
      servoL.write(60);
      servoR.write(120);
    }
  }
}

void glide_straight(float d_to_target, float t_heading1) {
  temp = zero_pitch(pitch);
  temp = conv_rads_to_degs( temp );
  input_pitch = temp;
  write_pitch( output_pitch );
  Serial.print("output_PID_pitch: ");
  Serial.println(output_pitch);
  temp = conv_rads_to_degs( roll );
  input_roll = temp;
  write_roll( output_roll );
  Serial.print("output_PID_roll: ");
  Serial.println(output_roll);
  servoL.write(servoValL);
  servoR.write(servoValR);
  Serial.print("Servo Data L: ");
  Serial.println(servoValL);
  Serial.print("Servo Data R: ");
  Serial.println(servoValR);
  videal = 16;
  if (d_to_target > 30)  snake();
  if (t_heading1 > 2) {
    //  turn left
    servoL.write(120);
    servoR.write(60);
  }
  if (t_heading1 < -2) {
    //  turn right
    servoL.write(60);
    servoR.write(120);
  }
}

void snake() {
  //  turn left
  servoL.write(120);
  servoR.write(60);
  delay(500);

  //  turn right
  servoL.write(60);
  servoR.write(120);
  delay(500);
}

double calc_pitch(double x, double y, double z) {
  double denom = pow(x, 2) + pow(z, 2);
  denom = sqrt(denom);
  double pitch = atan2(y, denom);
  return pitch;
}

double calc_roll(double x, double z) {
  double numen = -x;
  double roll = atan2(numen, z);
  return roll;
}

double filter_val(double curr, double prev, float alpha) {
  float a = 1;
  if (prev != -255) {
    a = alpha;
  }
  double filter_curr = a * curr;
  double filter_prev = (1 - a) * prev;
  return filter_curr + filter_prev;
}

double conv_rads_to_degs(double val) {
  double degs = val * 57296 / 1000;
  return degs;
}

double conv_degs_to_rads(double val) {
  double rads = val * 1000 / 57296;
  return rads;
}

void write_pitch(double pitch) {
  servoValL = map(pitch, -90, 90, 120, 60);
  servoValR = map(pitch, -90, 90, 120, 60);
}

double zero_pitch(double val) {
  val += OFFSET_PITCH;
  if ((val > -.02) && (val < .02)) {
    val = 0;
  }
  return val;
}

void write_roll(double roll) {
  double temp = map(roll, -180, 180, 0, 8);
  servoValL += temp;
  servoValR -= temp;
}

void get_imu_data() {
  stable_pitch.Compute();
  stable_roll.Compute();
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  Serial.print("grav: ");
  Serial.print("X: ");
  Serial.print(grav.x());
  Serial.print(" Y: ");
  Serial.print(grav.y());
  Serial.print(" Z: ");
  Serial.println(grav.z());
  Serial.println("\t\t");
  pitch = calc_pitch(grav.x(), grav.y(), grav.z());
  roll = calc_roll(grav.x(), -(grav.z()));
  pitch = filter_val(pitch, prev_pitch, ALPHA);
  roll = filter_val(roll, prev_roll, ALPHA);
  Serial.print("Pitch value: ");
  Serial.print(pitch);
  Serial.print("  Roll value: ");
  Serial.println(roll);
}

double distanceBetween(double lat1, double long1, double lat2, double long2)                                                                                                      // Update on 5-2-21 : 10:10PM
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1 - long2);
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
  double dlon = radians(long2 - long1);
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
