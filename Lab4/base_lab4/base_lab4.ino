/* Include necessary header files */
#include <Wire.h>
#include <MsTimer2.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

#include <SoftwareSerial.h>
SoftwareSerial BTserial(10, 13); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 10. 
// Connect the HC-06 RX to the Arduino TX on pin 13.

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
int sampling_rate = 200; // in milliseconds
unsigned long estimateTimer;

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
  Serial.begin(57600); //Open the serial port and set the baud rate
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

  BTserial.begin(57600);
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

void PD(){
  float Kp = 90;
  float Kd = 2;

  float errorTheta = 0.0 - theta;
  float errorThetaDot = 0.0 - thetaDot;
  
  float output = Kp*errorTheta + Kd*errorThetaDot;
  int outputScaled = -31.875*output;
  
  SetRightWheelSpeed(outputScaled);
  SetLeftWheelSpeed(outputScaled);
}

void LQR(){
  float K[4] = {-0.0500, -0.0341, 58.5291, 1.3097};

  float errorX = 0.0 - x;
  float errorXDot = 0.0 - xDot;
  float errorTheta = 0.0 - theta;
  float errorThetaDot = 0.0 - thetaDot;
  
  float output = K[0]*errorX + K[1]*errorXDot + K[2]*errorTheta + K[3]*errorThetaDot;
  int outputScaled = -output*1.5227*31.875;

  SetRightWheelSpeed(outputScaled);
  SetLeftWheelSpeed(outputScaled);
}

void sysIDLQR(){
//  float K[4] = {-0.3162, -0.1594, 40.5810, 1.9729};

  float K[4] = {-0.0500, -0.0341, 58.5291, 1.3097};

  float errorX = 0.0 - x;
  float errorXDot = 0.0 - xDot;
  float errorTheta = 0.0 - theta;
  float errorThetaDot = 0.0 - thetaDot;
  
  float output = K[0]*errorX + K[1]*errorXDot + K[2]*errorTheta + K[3]*errorThetaDot;

  float FREQS[5] = {1.8850, 3.3520, 5.9608, 10.5999, 18.8496};
  float freq = FREQS[4] / 1000;
  float amp = 4;

//  int outputScaled = -output*1.5227*31.875 - amp*sin(freq*time)*31.875;

  float randFloat = float(random(20000)-10000)/10000.0;

  int outputScaled = -output*1.5227*31.875 - amp*randFloat*31.875;

  randomInput = -amp*randFloat;

  SetRightWheelSpeed(outputScaled);
  SetLeftWheelSpeed(outputScaled);
}

/*
 The below code parses the data received from MATLAB
*/

const byte buffSize = 32;
char inputSeveral[buffSize]; // space for 31 chars and a terminator

byte maxChars = 12; // a shorter limit to make it easier to see what happens
                           //   if too many chars are entered
float data_MATLAB = 0.0;

float getdatafromMATLAB() 
{

    // Function credits: https://forum.arduino.cc/index.php?topic=236162.0

    // this function takes the characters from the serial input and converts them
    // to a single floating point value using the function "atof()"
     
    // a similar approach can be used to read an integer value if "atoi()" is used

    // first read severalChars into the array inputSeveral
    inputSeveral[0] = 0; 
    byte charCount = 0;  
    byte ndx = 0;
    
    if (BTserial.available() > 0) {
      long time = micros();
      while (BTserial.available() > 0) { 
        if (ndx > maxChars - 1) {
          ndx = maxChars;
        } 
        inputSeveral[ndx] = BTserial.read();
        ndx ++;        
        charCount ++;
      }
      if (ndx > maxChars) { 
        ndx = maxChars;
      }
      inputSeveral[ndx] = 0;

       // and then convert the string into a floating point number
     
      data_MATLAB = atof(inputSeveral); // atof gives 0.0 if the characters are not a valid number

      int outputScaled = data_MATLAB*31.875;
      SetLeftWheelSpeed(outputScaled);
      
//      Serial.print("Data from MATLAB -- ");
//      Serial.println(data_MATLAB, 3); // the number specifies how many decimal places
    }
}

/*
mainfunc() is the function that is called at your specified sampling rate. The default 
sampling rate is 5ms. This function will be called at every sampling instance.
*/
void mainfunc()
{
  /* Do not modify begins*/
  sei();
  time = millis(); 
  if (time - prev_time_encoder >= encoder_count_max)
  {
    readEncoder();
    prev_time_encoder = time;
  }
  
  readIMU();
  
  /* Do not modify ends*/
  /*Write your code below*/
  correctIMU();
  stateEstimate();

//  PD();
//  LQR();
//  sysIDLQR();

//  printCalIMU();

  BTserial.print("* ");
  BTserial.print(time);
  BTserial.print(" ");
  BTserial.print(theta);
  BTserial.print(" ");
  BTserial.print(thetaDot);
  BTserial.print(" ");
  BTserial.print(x);
  BTserial.print(" ");
  BTserial.print(xDot);
  BTserial.print(" ");
  BTserial.print(motor_left_ang_vel);
  BTserial.print(" ");
  BTserial.print(motor_right_ang_vel);
  BTserial.print(" ");
  BTserial.print(motor_left);
  BTserial.print(" ");
  BTserial.print(motor_right);
  BTserial.print(" *");
  BTserial.print("\n");

  getdatafromMATLAB();

//  BTserial.print(time);
//  BTserial.print(',');
//  BTserial.print(theta);
//  BTserial.print(',');
//  BTserial.print(thetaDot);
//  BTserial.print(',');
//  BTserial.print(x);
//  BTserial.print(',');
//  BTserial.print(xDot);
//  BTserial.print(',');
//  BTserial.print(motor_left_ang_vel);
//  BTserial.print(',');
//  BTserial.print(motor_right_ang_vel);
//  BTserial.print(',');
//  BTserial.print(motor_left);
//  BTserial.print(',');
//  BTserial.print(motor_right);
//  BTserial.print('\n');

  /***********************/
}

void loop()
{

}
