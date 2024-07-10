//PID library
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
MPU6050 mpu;
 
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//define a timer
unsigned long time1;

#define PID_MIN_LIMIT -150  //Min limit for PID 
#define PID_MAX_LIMIT 150 //Max limit for PID 
#define PID_SAMPLE_TIME_IN_MILLI 10  //This is PID sample time in milliseconds

//The pitch angle given by MPU6050 when robot is vertical and MPU6050 is horizontal is 0 in ideal case.
//However in real case its slightly off and we need add some correction to keep robot vertical.
//This is the angle correction to keep our robot stand vertically. Sometimes robot moves in one direction so we need to adjust this.
#define SETPOINT_PITCH_ANGLE_OFFSET -0.5
#define SETPOINT_YAW_ANGLE_OFFSET 0.1

#define MIN_ABSOLUTE_SPEED  20//Min motor speed 
//pitch variables:
double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;
//yaw variables:
double setpointYawAngle = SETPOINT_YAW_ANGLE_OFFSET;
double yawGyroAngle = 0;
double yawPIDOutput = 0;
//////////////// PID PARAMMETERS ////////////////////////////////////////////////////////////////////////////////////
// PICH parameters:
#define PID_PITCH_KP 14
#define PID_PITCH_KI 130
#define PID_PITCH_KD 0.8
// YAW parameters:
#define PID_YAW_KP 0.1
#define PID_YAW_KI 0.5
#define PID_YAW_KD 0.001

PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yawPID(&yawGyroAngle,&yawPIDOutput, &setpointYawAngle, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);

int enableMotor1=9;
int motor1Pin1=5;
int motor1Pin2=6;

int enableMotor2=10;
int motor2Pin1=8;
int motor2Pin2=7;

void setupPID()
{
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

  yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}
void setupMotors()
{
  pinMode(enableMotor1,OUTPUT);
  pinMode(motor1Pin1,OUTPUT);
  pinMode(motor1Pin2,OUTPUT);
  
  pinMode(enableMotor2,OUTPUT);
  pinMode(motor2Pin1,OUTPUT);
  pinMode(motor2Pin2,OUTPUT);

  rotateMotor(0,0);
}
void setupMPU()
{

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
//           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro 
//OFFSETS    1546,    -571,     834,     -19,       2,       2
  mpu.setXAccelOffset(1546); 
  mpu.setYAccelOffset(-571); 
  mpu.setZAccelOffset(834);   
  mpu.setXGyroOffset(-19);
  mpu.setYGyroOffset(2);
  mpu.setZGyroOffset(2);  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
  {
      // Calibration Time: generate offsets and calibrate our MPU6050
      // mpu.CalibrateAccel(6);
      // mpu.CalibrateGyro(6);
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();        
  } 
  else 
  {
      // ERROR!
  }
}

void setup() {
  Serial.begin(115200);
  //This is to set up motors
  setupMotors();   
  //This is to set up MPU6050 sensor
  setupMPU();
  //This is to set up PID 
  setupPID();
  //Initialize the bleutooth communication:
}
void loop()
{
  get_Values();
  pitchPID.Compute();
  rotateMotor(pitchPIDOutput+yawPIDOutput, pitchPIDOutput-yawPIDOutput);
  //print the data:
  // print_data();
}
void get_Values(){
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO. Get the Latest packet
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {  
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);
    yawGyroAngle = gy.z;                   //rotation rate in degrees per second
    pitchGyroAngle = ypr[1] * 180/M_PI;   //angle in degree
  }
}
void rotateMotor(int speed1, int speed2)
{
  if (speed1 < 0)
  {
    digitalWrite(motor1Pin1,LOW);
    digitalWrite(motor1Pin2,HIGH);   
  }
  else if (speed1>= 0)
  {
    digitalWrite(motor1Pin1,HIGH);
    digitalWrite(motor1Pin2,LOW);     
  }
  if (speed2 < 0)
  {
    digitalWrite(motor2Pin1,LOW);
    digitalWrite(motor2Pin2,HIGH);   
  }
  else if (speed2 >= 0)
  {   
    digitalWrite(motor2Pin1,HIGH);
    digitalWrite(motor2Pin2,LOW);  
  }
  speed1 = abs(speed1);
  speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, 150);

  speed2 = abs(speed2);
  speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, 150);
    
  analogWrite(enableMotor1,speed1);
  analogWrite(enableMotor2,speed2);    
}
void print_data(){
  // Serial.print("The gyro  before: ");
  Serial.print(pitchGyroAngle);
  Serial.print(" ");
  // Serial.print(" | The setpoints: ");
  Serial.print(setpointPitchAngle);
  Serial.print(" ");
  // Serial.print(" | The pid output: ");
  Serial.print(pitchPIDOutput);
  Serial.println();  
}