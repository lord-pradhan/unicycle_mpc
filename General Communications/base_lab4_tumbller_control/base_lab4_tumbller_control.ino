/* Include necessary header files */
#include <Wire.h>
#include <MsTimer2.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

#define BAUD 38400

#include <SoftwareSerial.h>
SoftwareSerial BTserial(10, 13); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 10. 
// Connect the HC-06 RX to the Arduino TX on pin 13.

// Communication to/from Matlab ------------

// Make sure that toMatlab and fromMatlab match with the Matlab script

struct __attribute__((packed)) {
  float dataTime;
  float robotState[5];
} toMatlab;

struct __attribute__((packed)) {
  int8_t startFlagFromMatlab;
  float dataTime;
  float voltInput;
  int16_t referenceInput[10];
} fromMatlab;

int toMatlabSize;
byte *toMatlabPtr;

int fromMatlabSize;
byte *fromMatlabPtr;

// ----------------------------------------

/* TB6612FNG motor driver signal pins */
#define IN1_L 7
#define IN1_R 12
#define PWM_L 5
#define PWM_R 6
#define STBY 8

/* Encoder count signal pins */
#define PinA_left 2
#define PinA_right 4

/* Sensor Reading Variables */
MPU6050 mpu; //Instantiate a MPU6050 object with the object name MPU.
int16_t ax, ay, az, gx, gy, gz; // Variables for IMU readings
volatile long right_encoder = 0;
volatile long left_encoder = 0;
/* 
 The volatile long type is used to ensure that the value is valid when the external 
 interrupt pulse count value is used in other functions 
 */
float num_of_encoder_counts_per_rev = 780.0;
float thousand_by_num_of_encoder_counts_per_rev = 1000.0/num_of_encoder_counts_per_rev;

/* IMU Callibration Variables */ 
long gyroXCalli = -105, gyroYCalli = 49, gyroZCalli = 77; // Obtained by callibrating gyroscope values
long accelXCalli = -1204, accelYCalli = 185, accelZCalli = 17120 - 16384; // Obtained by callibrating accelerometer values

//long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0; // Obtained by callibrating gyroscope values
//long accelXCalli = 0, accelYCalli = 0, accelZCalli = 0; // Obtained by callibrating accelerometer values

/* Motor PWM Input Values */
long motor_left = 0;
long motor_right = 0;

/* Time variables */
unsigned long time;
unsigned long prev_time_encoder = 0;
unsigned long startTime_left = 0;
unsigned long startTime_right = 0;
int sampling_rate = 5; // in milliseconds
unsigned long estimateTimer;

unsigned long interruptCount = 0;

/* Encoder to speed measurement variables */

int encoder_count_max = 20;
float motor_left_ang_vel = 0;
float motor_right_ang_vel = 0;

/* Robot parameters */

float wheel_rad = 0.0335; // radius of the wheel in metres
float lat_dist = 0.194; // distance between ends of the wheels in metres

/********************Initialization settings********************/
void setup() {
  
  /* TB6612FNGN Motor Driver module control signal initialization */
  pinMode(IN1_L, OUTPUT); //Control the direction of motor 1, 1 is forward, 0 is reverse
  pinMode(IN1_R, OUTPUT); //Control the direction of motor 1, 1 is forward, 0 is reverse
  pinMode(PWM_L, OUTPUT); //PWM of left motor
  pinMode(PWM_R, OUTPUT); //PWM of right motor
  pinMode(STBY, OUTPUT); //enable TB6612FNG
  
  /* Initializing motor drive module */
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN1_R, LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  
  /* Initialize I2C bus */
  Wire.begin(); //Add I2C bus sequence
  Serial.begin(115200); //Open the serial port and set the baud rate
  delay(1500);
  mpu.initialize(); //Initialization MPU6050
  delay(2);

  /* 
   Uncomment the next two lines only once and store the callibration values in the 
   global variables defined for callibration
  */
//  callibrateGyroValues();
//  callibrateAccelValues();
  
  time = millis();
  startTime_left = millis();
  startTime_right = millis();
  estimateTimer = millis();
  
  /* Interrupt function to count the encoder pulses */
  attachInterrupt(digitalPinToInterrupt(PinA_left), encoder_left, CHANGE);  
  /* 
  Timing interrupt settings, using MsTimer2. Because PWM uses a timer to control the 
  duty cycle, it is important to look at the pin port corresponding to the timer when 
  using timer.
  */
  MsTimer2::set(sampling_rate, mainfunc);
  MsTimer2::start();
  
  Serial.print("Setup Done!\n");

  // Communication to/from Matlab ------------

  toMatlabSize = sizeof(toMatlab);
  toMatlabPtr = (byte *)&toMatlab;
  
  fromMatlabSize = sizeof(fromMatlab);
  fromMatlabPtr = (byte *)&fromMatlab;

  BTserial.begin(BAUD);

  fromMatlab.startFlagFromMatlab = 0;

  while (BTserial.available()) {
    char junk = BTserial.read();
  }
  // ----------------------------------------
}

/***************************************************************************************/
/*
 This section contains functions that you can use to build your controllers
*/
/***************************************************************************************/

/* 
encoder_left() counts encoder pulses of the left wheel motor and stores it in a 
global variable 'left_encoder'
*/
void encoder_left(){left_encoder++;}
  
/* 
encoder_right() counts encoder pulses of the right wheel motor and stores it in a 
global variable 'right_encoder'
*/
void encoder_right(){right_encoder++;}


/* 
SetLeftWheelSpeed() takes one input which will be set as the PWM input to the left motor.
If the value is outside the range (-255,255), then the input will be saturated and the 
motor PWM will be set. The value is also written to the global variable 'motor_left'
*/
void SetLeftWheelSpeed(double speed_val)
{
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) { speed_val = 255; }
  else if (speed_val < -255) { speed_val = -255; }
  
  motor_left = speed_val;
  
  if (speed_val < 0)
  {
    digitalWrite(IN1_L, 1);
    analogWrite(PWM_L, -speed_val);
  }
  else
  {
    digitalWrite(IN1_L, 0);
    analogWrite(PWM_L, speed_val);
  }
}

/* 
SetRightWheelSpeed() takes one input which will be set as the PWM input to the right motor.
If the value is outside the range (-255,255), then the input will be saturated and the 
motor PWM will be set. The value is also written to the global variable 'motor_right'
*/
void SetRightWheelSpeed(double speed_val)
{
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) { speed_val = 255; }
  else if (speed_val < -255) { speed_val = -255; }
  
  motor_right = speed_val;
  
  if (speed_val < 0)
  {
    digitalWrite(IN1_R, 1);
    analogWrite(PWM_R, -speed_val);
  }
  else
  {
    digitalWrite(IN1_R, 0);
    analogWrite(PWM_R, speed_val);
  }
}

/*
readIMU() creates an MPU6050 class object and calls the function to read the six axis IMU.
The values are stored in the global variables ax,ay,az,gx,gy,gz where ax,ay,az are the 
accelerometer readings and gx,gy,gz are the gyroscope readings. 
*/
void readIMU()
{
  MPU6050 mpu_obj;
  mpu_obj.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
}

/*
readEncoder() takes the encoder pulse counts and calculates the angular velocities of the
wheels and stores it in the global variables 'motor_left_ang_vel' and 'motor_right_ang_vel'
*/
void readEncoder()
{ 
  // Encoder Calculations
  // angular velocity = (encoder_reading/num_of_counts_per_rotation)*(2*pi/sampling_time)
  
  motor_left_ang_vel = (float) 2 * 3.1415 * left_encoder * (float)thousand_by_num_of_encoder_counts_per_rev / (float)(time - startTime_left);  
  if (motor_left < 0){
    motor_left_ang_vel = -motor_left_ang_vel;}
  startTime_left = time;
  left_encoder = 0;  

  motor_right_ang_vel = motor_left_ang_vel;
  
//  motor_right_ang_vel = (float) 2 * 3.1415 * right_encoder * (float)thousand_by_num_of_encoder_counts_per_rev / (float)(time - startTime_right);  
//  if (motor_right < 0){
//    motor_right_ang_vel = -motor_right_ang_vel;}
//  startTime_right = time;
//  right_encoder = 0;  
}

/* 
printIMU() prints the IMU readings to the serial monitor in the following format:
ax,ay,az,gx,gy,gz
*/
void printIMU()
{
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print('\n');
}

/* 
printEncoder() prints the encoder readings to the serial monitor in the following format:
motor_left_ang_vel, motor_right_ang_vel
*/
void printEncoder()
{
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print('\n');
}

/* 
printAllData() prints the IMU readings, encoder readings and PWM inputs to the motor to
the serial monitor in the following format:
ax,ay,az,gx,gy,gz,motor_left_ang_vel,motor_right_ang_vel,motor_left,motor_right
*/
void printAllData()
{
  Serial.print(time);
  Serial.print(',');
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print(',');
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print(',');
  Serial.print(motor_left);
  Serial.print(',');
  Serial.print(motor_right);
  Serial.print('\n');
}

/*
callibrateGyroValues() gets the gyroscope readings n times and calculates the average of
the values to find the sensor bias. The callibration values are printed as:
gyroXCalli,gyroYCalli,gyroZCalli
*/
void callibrateGyroValues() 
{
    int n = 10000;
    for (int i=0; i < n; i++) 
    {
      readIMU();
      gyroXCalli = gyroXCalli + gx;
      gyroYCalli = gyroYCalli + gy;
      gyroZCalli = gyroZCalli + gz;
    }
    gyroXCalli = gyroXCalli/n;
    gyroYCalli = gyroYCalli/n;
    gyroZCalli = gyroZCalli/n;
    Serial.print(gyroXCalli);
    Serial.print(',');
    Serial.print(gyroYCalli);
    Serial.print(',');
    Serial.print(gyroZCalli);
    Serial.print('\n');
}

/*
callibrateAccelValues() gets the accelerometer readings n times and calculates the average of
the values to find the sensor bias. The callibration values are printed as:
accelXCalli,accelYCalli,accelZCalli
*/
void callibrateAccelValues() 
{
    int n = 10000;
    for (int i=0; i < n; i++) 
    {
      readIMU();
      accelXCalli = accelXCalli + ax;
      accelYCalli = accelYCalli + ay;
      accelZCalli = accelZCalli + az;
    }
    accelXCalli = accelXCalli/n;
    accelYCalli = accelYCalli/n;
    accelZCalli = accelZCalli/n;
    Serial.print(accelXCalli);
    Serial.print(',');
    Serial.print(accelYCalli);
    Serial.print(',');
    Serial.print(accelZCalli);
    Serial.print('\n');
}

/***************************************************************************************/
/***************** Write your custom variables and functions below *********************/
/***************************************************************************************/

float Ax, Ay, Az, Gx, Gy, Gz;
float theta, thetaDot, x, xDot;
float randomInput;

void correctIMU(){
  Ax = (float(ax - accelXCalli)*9.81)/16384.0;
  Ay = (float(ay - accelYCalli)*9.81)/16384.0;
  Az = (float(az - accelZCalli)*9.81)/16384.0;
  
  Gx = (float(gx - gyroXCalli)*PI)/(131.0*180.0);
  Gy = (float(gy - gyroYCalli)*PI)/(131.0*180.0);
  Gz = (float(gz - gyroZCalli)*PI)/(131.0*180.0);  
}

void printCalIMU(){
  Serial.print(Ax);
  Serial.print(',');
  Serial.print(Ay);
  Serial.print(',');
  Serial.print(Az);
  Serial.print(',');
  Serial.print(Gx);
  Serial.print(',');
  Serial.print(Gy);
  Serial.print(',');
  Serial.print(Gz);
  Serial.print('\n');  
}

void stateEstimate(){
  long elapsedTime = millis() - estimateTimer;
  
  float alpha = 0.97;
  theta = alpha*(theta + ((Gx*float(elapsedTime))/1000.0)) + (1.0 - alpha)*atan(Ay/sqrt(pow(Ax,2.0) + pow(Az,2.0)));

  thetaDot = Gx;

  xDot = motor_left_ang_vel*0.0335;
  x = x + (xDot*elapsedTime)/1000.0;

  estimateTimer = millis();  
}

void LQR(float ref[4]){
//  float K[4] = {10.00, 0.0341, 58.5291, 1.3097};
  float K[4] = {-5.0000, -2.1038, 56.1637, 1.8112};
  
  float output = K[0]*(ref[0] - x) + K[1]*(ref[1] - xDot) + K[2]*(ref[2] - theta) + K[3]*(ref[3] - thetaDot);
  int outputScaled = -output*1.5227*31.875;

  SetRightWheelSpeed(outputScaled);
  SetLeftWheelSpeed(outputScaled);
}

  // Communication to/from Matlab ------------

void writeMatlab() {
  toMatlab.dataTime = float(time)*1e-3;
  toMatlab.robotState[0] = x;
  toMatlab.robotState[1] = xDot;
  toMatlab.robotState[2] = theta;
  toMatlab.robotState[3] = thetaDot;
  toMatlab.robotState[4] = float(motor_left)/31.875;
  
  BTserial.write((byte *)&toMatlab,toMatlabSize);
}

char  outString[120];

void readMatlab() {
  if (BTserial.available() > 0) {
    int ptrCount = 0;
    while (BTserial.available() > 0) {
      unsigned char temp = BTserial.read();
      if (ptrCount < sizeof(fromMatlab)) {
        *(fromMatlabPtr+ptrCount) = temp;
        ptrCount++;
      } 
    }
//    Serial.print(fromMatlab.startFlagFromMatlab);
//    Serial.print(',');
//    Serial.print(fromMatlab.dataTime);
//    Serial.print(',');
//    Serial.print(fromMatlab.voltInput);
//    Serial.print(',');
//    for (int i = 0; i < 10; i++) {
//      Serial.print(fromMatlab.referenceInput[i]);
//      Serial.print(',');
//    }
//    Serial.print('\n');
  }
}

  // ----------------------------------------

/*
mainfunc() is the function that is called at your specified sampling rate. The default 
sampling rate is 5ms. This function will be called at every sampling instance.
*/

float reference[4];

void mainfunc()
{
  // execute this every 5 ms
  sei();
  interruptCount += sampling_rate;

  time = millis(); 
  if (time - prev_time_encoder >= encoder_count_max)
  {
    readEncoder();
    prev_time_encoder = time;
  }
  readIMU();
  correctIMU();
  stateEstimate();

  readMatlab();

  // Scale here is 1000x for int16 data type
  if (fromMatlab.startFlagFromMatlab == 1) {
    if (fromMatlab.referenceInput[0] < 500 && fromMatlab.referenceInput[0] > -500) {
      reference[0] = float(fromMatlab.referenceInput[0])*1e-3; 
    }
  } 
  
  reference[1] = 0.0;
  reference[2] = 0.0;
  reference[3] = 0.0;
  
  LQR(reference);

  // execute this every 200 ms
  if (interruptCount%200 == 0) {
    if (fromMatlab.startFlagFromMatlab == 1) {
      writeMatlab(); 
    }
  }


}

void loop()
{

}
