#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Set servo start value 90 degrees
const int initialServoValue = 90;

// Declare servos
Servo servo1;
Servo servo2;

// Initialize PID algorithm
double desired, input, output;
double kp = 1.25;
double ki = 0.04;
double kd = 0.1;
PID pid(&input, &output, &desired, kp, ki, kd, DIRECT);

// Initialize BNO055 gyro sensor
Adafruit_BNO055 bno(55);

// Timestamp tracker for serial helper functions (PID Frontend)
unsigned long serialTime = 0;


/*************************************************************
 * Serial Communication functions / helpers for PID Frontend *
 *************************************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
values;                // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index = 0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available() && index < 26)
  {
    if (index == 0) {
      Auto_Man = Serial.read();
    } else if (index == 1) {
      Direct_Reverse = Serial.read();
    } else {
      values.asBytes[index - 2] = Serial.read();
    }
    index++;
  }

  // if the information we got was in the correct format,
  // read it into the system
  if(index == 26 && (Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
  {
    desired = double(values.asFloat[0]);
    //input=double(values.asFloat[1]);       // * the user has the ability to send the
                                          //   value of "Input"  in most cases (as
                                          //   in this one) this is not needed.
    if(Auto_Man == 0)                       // * only change the output if we are in
    {                                     //   manual mode.  otherwise we'll get an
      output = double(values.asFloat[2]);      //   output blip, then the controller will
    }                                     //   overwrite.

    double p, i, d;                       // * read in and set the controller tunings
    p = double(values.asFloat[3]);           //
    i = double(values.asFloat[4]);           //
    d = double(values.asFloat[5]);           //
    pid.SetTunings(p, i, d);            //

    if (Auto_Man == 0) {
      pid.SetMode(MANUAL);// * set the controller mode
    } else {
      pid.SetMode(AUTOMATIC);
    }            //

    if (Direct_Reverse == 0) {
      pid.SetControllerDirection(DIRECT); // * set the controller Direction
    } else {
      pid.SetControllerDirection(REVERSE);
    }          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(desired);
  Serial.print(" ");
  Serial.print(input);
  Serial.print(" ");
  Serial.print(output);
  Serial.print(" ");
  Serial.print(pid.GetKp());
  Serial.print(" ");
  Serial.print(pid.GetKi());
  Serial.print(" ");
  Serial.print(pid.GetKd());
  Serial.print(" ");

  if (pid.GetMode() == AUTOMATIC) {
    Serial.print("Automatic");
  } else {
    Serial.print("Manual");
  }
  Serial.print(" ");

  if (pid.GetDirection() == DIRECT) {
    Serial.println("Direct");
  } else {
     Serial.println("Reverse");
  }
}


/********************
 * Now system stuff *
 ********************/

void setup() {
  // Start serial connection
  Serial.begin(9600);

  // Trap application into endless loop, if gyro sensor fails
  if (!bno.begin()) {
    Serial.println("Error initializing gyro sensor.\nPlease check connection!");
    while(1);
  }

  delay(200);
  bno.setExtCrystalUse(true);

  // Set desired PID value to 0 degrees
  desired = 0;

  // Define servo data pins (PWM enabled)
  servo1.attach(9);
  servo2.attach(10);

  // Set both servos to 90 degrees
  servo1.write(initialServoValue);
  servo2.write(initialServoValue);
  
  // Initialize PID
  pid.SetMode(AUTOMATIC);
}

void loop() {
  // Read gyro sensor data
  sensors_event_t event;
  bno.getEvent(&event);

  // Get current degrees for Z axis
  int degreeZ = event.orientation.z;
  input = abs(degreeZ);               // Calculate to absolute degrees
  pid.Compute();                      // Get correction value for Z axis

  // Apply correction value for Z axis
  int newZ;
  if (degreeZ < 0) {
    newZ = degreeZ + output;          // If gyro was moved counter clock-wise, add correction value
  } else {
    newZ = degreeZ - output;          // If gyro was moved clock-wise, subtract correction value
  }

  // Limit servo to min 0, or max 180 degrees
  if (newZ < -90) {
    newZ = -90;
  } else if (newZ > 90) {
    newZ = 90;
  }
  servo1.write(newZ + initialServoValue); // Add initialServoValue to make sure Servo is between 0 and 180

  // Communication functions to PID Frontend
  // for tuning PID values and reading out current PID calculations
  if (millis() > serialTime) {
    SerialReceive();
    SerialSend();
    serialTime += 500;  // Call functions again in 500ms
  }
}
