#include <Wire.h>
#include <SoftwareSerial.h>
//loads the software serial library, creates a new serial port on desired pins
#include <Servo.h>

Servo vertical; //create servo object

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536
#define M_PI 3.14159265359      
#define dt 0.01             // 10 ms sample rate!   

void setup() {
//Servo setup
vertical.attach(5); //set vertical control pin5 //servo range is 40 - 140
Serial.begin(57600);
//IMU Setup
Wire.begin();
setupMPU();
}
void loop() {
//Local variable declarations:
Serial.println("Starting here");
float pitch;
while (1)
{
pitch = recordAccelRegisters();
verticalTurn(pitch);
Serial.println("----------begin debug data:-----------");
Serial.print("pitch (deg): ");
Serial.println(pitch);
Serial.println("-----------end debug data:------------");
}
}
//-----------code that controls the servos--------------
void verticalTurn(float pitch)
{
float trim = 0; //this value can be used to adjust the trim, add numbers to make it go down more, subtract to make it go up. (should be SMALL!) like .01
float verticalAdjustmentIntensity = 1.0;
double adjustment = 90.0;
int debug = 0;
pitch = verticalAdjustmentIntensity * pitch + trim;
if (pitch < 0 && pitch > -1.05) { //glider is pointing down so we need to go up
adjustment = 90.0 - (50 * -1.0 * pitch);
vertical.write(adjustment);
Serial.println("pointing down");
//Serial.println(adjustment);
}
else if (pitch > 0 && pitch < 1.05) //glider is pointing up so we need to go down
{
adjustment = 90.0 + (50 * pitch);
vertical.write(adjustment);
Serial.println("pointing up");
}
if (debug)
{
Serial.print("adjustment value");
Serial.println(adjustment);
delay(500);
}
}
//-----------code that controls the IMU----------------
void setupMPU() {
Wire.beginTransmission(0b1101000);//I2C adress of the MPU
Wire.write(0x6B); //6B in the register map is the power mode of the IMU
Wire.write(0b00000000); //when we write a zero it will not allow the IMU to sleep
Wire.endTransmission();
//Gyro setup
//Wire.beginTransmission(0b1101000);//I2C address of MPU
//Wire.write(0x1B);
//Wire.write(0b00000000); //Set the gyro to full scale
//Wire.endTransmission();
//Accelerometer setup
Wire.beginTransmission(0b1101000);//I2C address of MPU
Wire.write(0x1C);
Wire.write(0b00000000); //set the accelerometer to +/- 2G
Wire.endTransmission();
//Magnatometer setup
//Wire.beginTransmission(0b1101000);//I2C address of MPU
//Wire.write(0x0A);
//Wire.write(0b00010010); //set the magnetometer to be read in continuous one mode
//Wire.write(0b00010110); //set the magnetometer to be read in continuous two mode
//Wire.endTransmission();
}
float recordAccelRegisters() {
Wire.beginTransmission(0b1101000); //I2C address of the MPU
Wire.write(0x3B); //start register for Accel Readings
Wire.endTransmission();
Wire.requestFrom(0b1101000, 6); //request Accel registers (3B - 40)
while (Wire.available() <6);
long accelX = Wire.read() << 8 | Wire.read(); //store the first to bytes x
long accelY = Wire.read() << 8 | Wire.read(); //store the first to bytes Y
long accelZ = Wire.read() << 8 | Wire.read(); //store the first to bytes Z
float pitch = processAccelData(accelX, accelY, accelZ);
return pitch;
}
float processAccelData(long accelX, long accelY, long accelZ) {
float gForceX = accelX / 16384.0;
float gForceY = accelY / 16384.0;
float gForceZ = accelZ / 16384.0;
return gForceY;
}

void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
 
    // integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
    // compensate for drift with accelerometer data if !bullshit
    // sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
  // turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
  // turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
} 
