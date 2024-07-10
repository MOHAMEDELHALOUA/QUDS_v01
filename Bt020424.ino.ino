//PID library
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
//Include Bleutooth library
#include <SoftwareSerial.h>


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

#define PID_MIN_LIMIT -255  //Min limit for PID 
#define PID_MAX_LIMIT 255 //Max limit for PID 
#define PID_SAMPLE_TIME_IN_MILLI 10  //This is PID sample time in milliseconds

//The pitch angle given by MPU6050 when robot is vertical and MPU6050 is horizontal is 0 in ideal case.
//However in real case its slightly off and we need add some correction to keep robot vertical.
//This is the angle correction to keep our robot stand vertically. Sometimes robot moves in one direction so we need to adjust this.

int MotorAspeed, MotorBspeed;
float MOTORSLACK_A = 40;                 // Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B = 40;

double setpoint = 1;
double newSetpoint;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;

double Ysetpoint = 0.1;
double yoriginalSetpoint;
double yGyroAngle = 0;
double yOutput = 0;

//////////////// PID PARAMMETERS ////////////////////////////////////////////////////////////////////////////////////
#define PID_PITCH_KP 11
#define PID_PITCH_KI 110
#define PID_PITCH_KD 0.6

#define PID_YAW_KP 10
#define PID_YAW_KI 60
#define PID_YAW_KD 0.1

PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &newSetpoint, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yawPID(&yGyroAngle, &yOutput, &Ysetpoint, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);
int enableMotor1=10;
int motor1Pin1=6;
int motor1Pin2=5;

int motor2Pin1=8;
int motor2Pin2=7;
int enableMotor2=9;

#define d_speed 1.5
#define d_dir 3

void setupPID()
{
  newSetpoint = setpoint;
  Ysetpoint = yoriginalSetpoint;
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}
///////////////////// Bluetooth //////////////////////////////////////////////////////////////////////////////////
//creat an object for softwareSerial:
const int rxpin = 3;       //Bluetooth serial stuff
const int txpin = 4;
SoftwareSerial blue(rxpin, txpin);
SoftwareSerial mySerial(rxpin,txpin);//3 and 4 are the pins where i connect the TXD and RXD.
char cmdB;

void setupMotors()
{
  pinMode(enableMotor1,OUTPUT);
  pinMode(motor1Pin1,OUTPUT);
  pinMode(motor1Pin2,OUTPUT);
  
  pinMode(enableMotor2,OUTPUT);
  pinMode(motor2Pin1,OUTPUT);
  pinMode(motor2Pin2,OUTPUT);

  rotateMotor(0, 0);
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
//OFFSETS    -122,     111,    800,    17,      -32,      38
  mpu.setXAccelOffset(1592); 
  mpu.setYAccelOffset(-535); 
  mpu.setZAccelOffset(820);   
  mpu.setXGyroOffset(-14);
  mpu.setYGyroOffset(7);
  mpu.setZGyroOffset(0);  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
  {
      // Calibration Time: generate offsets and calibrate our MPU6050
      //mpu.CalibrateAccel(6);
      //mpu.CalibrateGyro(6);
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

void setup(){

  Serial.begin(115200);
  mySerial.begin(9600);
  pinMode(rxpin,INPUT);
  pinMode(txpin,OUTPUT);
  //This is to set up motors
  setupMotors();   
  //This is to set up MPU6050 sensor
  setupMPU();
  //This is to set up PID 
  setupPID();
  //Initialize the bleutooth communication:
  mySerial.begin(115200);
  if(mySerial.available()){
    mySerial.println("the device is ready.");
  }
  //Initiasize time
  time1 = millis();
}
void loop()
{
  get_Values();
  Bluetooth_control();
  print_data();
}
void newPID()
{
  pitchPID.Compute();
  yawPID.Compute();
  // Convert PID output to motor control
  MotorAspeed = compensate_slack(yOutput, pitchPIDOutput, 1);
  MotorBspeed = compensate_slack(yOutput, pitchPIDOutput, 0);
  //change speed
  rotateMotor(MotorAspeed, MotorBspeed);
}
// Bluetooth control function
void Bluetooth_control(){
  if(mySerial.available()){
	cmdB = mySerial.read();
	if (cmdB == "F") {
		newSetpoint = setpoint - d_speed; //Forward
	} else if (cmdB == "B") {
		newSetpoint = setpoint + d_speed; //Backward
	} else if (cmdB == "L") {
		Ysetpoint = constrain((Ysetpoint + yoriginalSetpoint - d_dir), -180, 180); //Left
	} else if (cmdB == "R") {
		Ysetpoint = constrain((Ysetpoint + yoriginalSetpoint + d_dir), -180, 180); //Right
	} else if (cmdB == "S") {
		newSetpoint = setpoint; // Stay in the original point
	} else {
		//unvalide
	}
   }
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

    pitchGyroAngle = ypr[1] * 180/M_PI;   //angle in degree
  }
}
double compensate_slack(double yOutput, double pitchPIDOutput, bool A) {
  // Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
  //yOutput is for left,right control
  if (A)
  {
    if (pitchPIDOutput >= 0)
      pitchPIDOutput = pitchPIDOutput + MOTORSLACK_A - yOutput;
    if (pitchPIDOutput < 0)
      pitchPIDOutput = pitchPIDOutput - MOTORSLACK_A - yOutput;
  }
  else
  {
    if (pitchPIDOutput >= 0)
      pitchPIDOutput = pitchPIDOutput + MOTORSLACK_B + yOutput;
    if (pitchPIDOutput < 0)
      pitchPIDOutput = pitchPIDOutput - MOTORSLACK_B + yOutput;
  }
  pitchPIDOutput = constrain(pitchPIDOutput , PID_MIN_LIMIT, PID_MAX_LIMIT);
  return pitchPIDOutput;
}

void rotateMotor(int MotorAspeed, int MotorBpeed)
{
  if (MotorAspeed >= 0)
  {
    digitalWrite(motor1Pin1,LOW);
    digitalWrite(motor1Pin2,HIGH);    
  }
  else if (MotorAspeed < 0)
  {
    digitalWrite(motor1Pin1,HIGH);
    digitalWrite(motor1Pin2,LOW);      
  }

  if (MotorBpeed >= 0)
  {
    digitalWrite(motor2Pin1,LOW);
    digitalWrite(motor2Pin2,HIGH);    
  }
  else if (MotorBpeed < 0)
  {
    digitalWrite(motor2Pin1,HIGH);
    digitalWrite(motor2Pin2,LOW);      
  }

    
  analogWrite(enableMotor1,MotorAspeed);
  analogWrite(enableMotor2,MotorBpeed);    
}
void print_data(){
  // Serial.print("The gyro  before: ");
  Serial.print(pitchGyroAngle);
  Serial.print(" ");
  // Serial.print(" | The setpoints: ");
  Serial.print(setpoint);
  Serial.print(" ");
  // Serial.print(" | The pid output: ");
  Serial.print(pitchPIDOutput);
  //yaw angle:
  Serial.print(yGyroAngle);
  Serial.print(" ");
  //yaw setpoint:
  Serial.print(Ysetpoint);
  Serial.print(" ");
  // yaw output:
  Serial.print(yOutput);
  Serial.print(" ");
  Serial.println();  
}