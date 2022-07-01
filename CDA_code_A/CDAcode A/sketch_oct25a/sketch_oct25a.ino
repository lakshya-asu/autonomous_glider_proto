#include <Wire.h>
#include <SoftwareSerial.h>
#include <MPU9250.h>
#include <TinyGPS++.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h>
#include <Servo.h>

TinyGPSPlus gps;
SoftwareSerial xbee(2,3); //Tx Rx
SoftwareSerial GPS(4,5);
MPU9250 IMU(Wire,0x68);

///////////////////////////////IMU Stuff///////////////////////////////////
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
float roll, pitch, heading;
///////////////////////////////IMU Stuff///////////////////////////////////

/////////////////////////////////PID Stuff////////////////////////////////////////
double Kp=1, Ki=1, Kd=0.07;  //PID Tuning Parameters (NOTE: idk if I tuned it properly)
//     Roll PID stuff
double rollSetpoint, rollInput, rollOutput;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);

//   PitchPID Stuff
double pitchSetpoint, pitchInput, pitchOutput;
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

//Specify the links and initial tuning parameters
/////////////////////////////////PID Stuff////////////////////////////////////////

Servo leftServo, rightServo; //Servos for flaps
int leftOutput, rightOutput; //Output for flaps

boolean configure;
int Status;
float gxb,gyb,gzb,axb,ayb,azb;
float gxd,gyd,gzd,axd,ayd,azd;
float Lat,Lon,Altitude;
float T_Lat, T_Lon;
float heading;
float prev_gx,prev_gy,prev_gz,prev_ax,prev_ay,prev_az,prev_time;
float angle_x,angle_y,angle_z;
int HMC6352Address = 0x42;
int slaveAddress;
byte headingData[2];
int i, headingValue
int headingcompass;


void set_last_value(int Time,float gxp,float gyp,float gzp,float axp,float ayp,float azp);
void complimentary();
void MPUcalibration();
void MPUdata();
boolean RFconfiguration();
void gpsdata();

void setup() 
{
  Serial.begin(9600);
  xbee.begin(9600);
  GPS.begin(9600);
  configure = RFconfiguration();
  MPUcalibration();
  set_last_value(millis(),0,0,0,0,0,0);

      ///////////////PID Setup/////////////
      rollInput = roll; //Sets the PID inputs to the gyro's roll and pitch axes
      pitchInput = pitch;
    
      rollSetpoint = 0; 
      pitchSetpoint = 0;
    
      rollPID.SetOutputLimits(-90, 90); //limits the PID loop to output numbers in between -90 and 90. I used values from -90 to 90 in order to better visualize the glider position(when flaps are flat, the PID output is 0)
      pitchPID.SetOutputLimits(-90,90);
    
      rollPID.SetMode(AUTOMATIC);  //Turns PID on
      pitchPID.SetMode(AUTOMATIC);
    ///////////////PID Setup/////////////

    ///////////////IMU Setup/////////////      
      //start filter
      filter.begin(25);
      // initialize variables to pace updates to correct rate
      microsPerReading = 1000000 / 25;
      microsPrevious = micros();
    ///////////////IMU Setup/////////////

      leftServo.attach(9);
      rightServo.attach(10); 
}

void loop() 
{
  int t_on;
  t_on = millis();
  gpsdata();
  MPUdata();
  set_last_value(t_on,gxd,gyd,gzd,axd,ayd,azd);
  complimentary();
  Serial.println(angle_x);
  Serial.println(angle_y);
  Serial.println(angle_z);
  delay(1);
}

boolean RFconfiguration() 
{
  // Set the data rate for the SoftwareSerial port:
  xbee.begin(9600);      
  // Put the radio in command mode:
  Serial.write("Entering command mode\r");
  delay(1000);
  while(xbee.available()>0)
    xbee.read();
  xbee.write("+++");
  while(xbee.available()>0) 
     xbee.read();
  //delay(1000);
  //while(xbee.available() > 0) {Serial.write(xbee.read());}    
  String ok_response = "OK\r"; // The response we expect
  // Read the text of the response into the response variable
  // This satisfies the guard time by waiting for the OK message
  String response = String("");
  while (response.length() < ok_response.length()) 
  {
     if (xbee.available() > 0) 
     {
        response += (char) xbee.read();
     }
  }
  Serial.println("response1: " + response);
  // If we got received OK, configure the XBee and return true:
  if (response.equals(ok_response)) 
  {
    Serial.println("Enter command mode successful"); 
    // Restore to default values:
    Serial.println("Restoring default values before making changes");
    xbee.write("ATRE\r");
    Serial.println("Setting addr high");
    xbee.write("ATDH0\r");  // Destination high
    //while(xbee.available() > 0) {Serial.write(xbee.read());}
    Serial.println("Setting addr low");
    xbee.write("ATDL1\r");  // Destination low-REPLACE THIS
    //while(xbee.available() > 0) {Serial.write(xbee.read());}
    Serial.println("Setting MY address");
    xbee.write("ATMYFFFF\r");    
    // Apply changes:
    Serial.println("Applying changes");
    xbee.write("ATAC\r");
    Serial.write("Exit command mode\r");
    xbee.write("ATCN\r");  // Exit command mode
    //while(xbee.available() > 0) {Serial.write(xbee.read());}
    Serial.write("Finished\r");
    return true;
  } 
  else 
    return false; // This indicates the response was incorrect
}
void MPUcalibration()
{
  int statusg,statusa;
  Status = IMU.begin();
  if (Status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(Status);
    Serial.println("Reset system");
    while(1) 
      {}
  }
   // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 1 for a 500 Hz update rate
  IMU.setSrd(1);    // OUTPUT data rate = 1000/(1-SRD) 3.9Hz to 1KHz
  statusg = IMU.calibrateGyro();
  gxb = IMU.getGyroBiasX_rads();
  gyb = IMU.getGyroBiasY_rads();
  gzb = IMU.getGyroBiasZ_rads();
  Serial.print("gxb = ");
  Serial.print(gxb);
  Serial.print(" gyb = ");
  Serial.print(gyb);
  Serial.print(" gzb = ");
  Serial.println(gzb);
  
  statusa = IMU.calibrateAccel();
  axb = IMU.getAccelBiasX_mss();
  ayb = IMU.getAccelBiasY_mss();
  azb = IMU.getAccelBiasZ_mss();
  Serial.println("axb = ");
  Serial.print(axb);
  Serial.print(" ayb = ");
  Serial.print(ayb);
  Serial.print(" azb = ");
  Serial.println(azb);
  Serial.println("MPU9250 Calibrated");
}
void MPUdata()
{
  IMU.readSensor();
  axd = IMU.getAccelX_mss();
  ayd = IMU.getAccelY_mss();
  azd = IMU.getAccelZ_mss();
  gxd = IMU.getGyroX_rads();
  gyd = IMU.getGyroY_rads();
  gzd = IMU.getGyroZ_rads();
  Serial.println(axd);
  Serial.println(ayd);
  Serial.println(azd);
  Serial.println(gxd);
  Serial.println(gyd);
  Serial.println(gzd);
}
void gpsdata()
{
  while (GPS.available()>0) // check for gps data
  {
    gps.encode(GPS.read()); // reads the gps data
    if (gps.location.isUpdated())  // check for update in gps location data
    { 
      Lat = gps.location.lat();
      Lon = gps.location.lng();
      Serial.print("Latitude="); // Prints latitude
      Serial.print(Lat,6); // prints latitude data
      Serial.print("Longitude=");// Prints longitude
      Serial.println(Lon,6); // prints longitude data 
    }
    if (gps.altitude.isUpdated()) // check for update in gps location data
    { 
      Altitude = gps.altitude.meters(); // prints altitude data
      Serial.print("Altitude="); // Prints altitude
      Serial.print(Altitude);
      Serial.println("M");
    }
  }
}

void set_last_value(int Time,float gxp,float gyp,float gzp,float axp,float ayp,float azp)
{
  prev_gx = gxp;
  prev_gy = gyp;
  prev_gz = gzp;
  prev_ax = axp;
  prev_ay = ayp;
  prev_az = azp;
  prev_time = Time;
}

void complimentary()
{
  float T,delt,dt;
  float t_now = millis();
  dt = (t_now - prev_time)/1000;
  float gyro_angle_x = gxd*dt+prev_gx;
  float gyro_angle_y = gxd*dt+prev_gy;
  float gyro_angle_z = gxd*dt+prev_gz;
  T = 1;
  delt = 1;
  float alpha = T/(T + delt);
  angle_x = alpha*gyro_angle_x + (1.0 - alpha)*axd;
  angle_y = alpha*gyro_angle_y + (1.0 - alpha)*ayd;
  angle_z = gyro_angle_z;
}

void PIDStabilizer()
{
  rollInput = roll; //sets PID inputs to Roll and Pitch
  pitchInput = pitch;
  rollPID.SetTunings(Kp, Ki, Kd); //PID Tunings
  pitchPID.SetTunings(Kp, Ki, Kd);

  float ax, ay, az;
  float gx, gy, gz;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    ax=axd;ay=ayd; az=azd; gx=gxd; gy=gyd; gz=gzd;
    
    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();



    //Calculates PID
    rollPID.Compute();
    pitchPID.Compute();

    //Combines the outputs and adds 90 to make it work with the servos
    leftOutput = ((rollOutput + pitchOutput) / 2) + 90;
    rightOutput = ((rollOutput - pitchOutput) / 2) + 90;

    

     *  One issue with the code is that the way I mix the Roll and Pitch PID loops
     * is not the best way to mix PID loops. For instance, if the pitchPID was 90, and 
     * rollPID was 0, both flaps would not trim up, rather, rightServo would be at -45,
     * while leftServo trims at 135.  
     * 
  
     */

     
    //writes to the servos
    leftServo.write(leftOutput);
    rightServo.write(rightOutput);
    Serial.println(pitch);
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  } //if
  }
}
}

//----------------------------------------------------------------//
//------------------------guiding funtions------------------------//
//----------------------------------------------------------------//

void distancecalc(){
 float flat1=flat;     // flat1 = our current latitude. flat is from the gps data. 
 float flon1=flon;  // flon1 = our current longitude. flon is from the fps data.
float dist_calc=0;
float dist_calc2=0;
float diflat=0;
float diflon=0;
x2lat=      ;  //enter a latitude point here   this is going to be your waypoint
x2lon=      ;  // enter a longitude point here  this is going to be your waypoint
//------------------------------------------ distance formula below. Calculates distance from current location to waypoint
diflat=radians(x2lat-flat1);  //notice it must be done in radians
flat1=radians(flat1);    //convert current latitude to radians
x2lat=radians(x2lat);  //convert waypoint latitude to radians
diflon=radians((x2lon)-(flon1));   //subtract and convert longitudes to radians
dist_calc = (sin(diflat/2.0)sin(diflat/2.0));
dist_calc2= cos(flat1);
dist_calc2=cos(x2lat);
dist_calc2*=sin(diflon/2.0);                                       
dist_calc2*=sin(diflon/2.0);
dist_calc +=dist_calc2;
dist_calc=(2atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
dist_calc=6371000.0; //Converting to meters
Serial.println(“distance from target”);
Serial.println(dist_calc);    //print the distance in meters
}

void headingcalc(){
 flon1 = radians(flon1);  //also must be done in radians
 x2lon = radians(x2lon);  //radians duh.
 heading = atan2(sin(x2lon-flon1)*cos(x2lat),cos(flat1)*sin(x2lat)-sin(flat1)cos(x2lat)cos(x2lon-flon1)),23.1415926535;
  heading = heading180/3.1415926535;  // convert from radians to degrees
  int head =heading; //make it a integer now
  if(head<0){
  heading+=360;   //if the heading is negative then add 360 to make it positive
}
  Serial.println(“heading:”);
  Serial.println(heading);   // print the heading.
  Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
  Wire.send(“A”);              
  Wire.endTransmission();
  delay(10);                  
  Wire.requestFrom(slaveAddress, 2);       
  i = 0;
  while(Wire.available() && i < 2)
  { 
    headingData[i] = Wire.receive();
    i++;
  }
  headingValue = headingData[0]*256 + headingData[1];
 int pracheading = headingValue / 10;      // this is the heading of the compass
 if(pracheading>0){
   headingcompass=pracheading;
 }
  Serial.println(“current heading:”);
  Serial.println(headingcompass);
  x4=headingcompass-heading;   //getting the difference of our current heading to our needed bearing
  Serial.println(“turning angle:”,x4);
}

 int guidingfunction(){
  int turn;
//-------------------------------------- below tells us which way we need to turn
if(x4>=-180){
  if(x4<=0){
    turn=8;    //set turn =8 which means “right”         
  }
}
if(x4<-180){
  turn=5;      //set turn = 5 which means “left”
}
if(x4>=0){
  if(x4<180){
    turn=5;   //set turn = 5 which means “left”
  }
}
if(x4>=180){     //set turn =8 which means “right”
  turn=8;
}
int hd = headingcompass;
if(hd==heading){
    turn=3;   //then set turn = 3 meaning go “straight”
}

//executing turn
if(turn==3){
  Serial.println(“straight”);
}
//-------------------------------------------------------------------------------------turn right
if(turn==8){
rightturn();
}
//------------------------------------------------------------------------------------------turn left
if(turn==5){
leftturn();
}
