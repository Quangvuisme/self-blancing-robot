#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 70
//bluetooth
const int rxpin =13;
const int txpin =12;
SoftwareSerial mySerial(rxpin,txpin);
char content1;
String content;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[128]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 183;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
double output_left;
double output_right;
double factor_left=1;
double factor_right=1;

//adjust these values to fit your own design
double Kp = 30;   
double Kd = 2;
double Ki = 150;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double originalMotorSpeedFactorLeft = 0.75;
double originalMotorSpeedFactorRight = 0.6;
double motorSpeedFactorLeft = originalMotorSpeedFactorLeft;
double motorSpeedFactorRight = originalMotorSpeedFactorRight;

//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 9;
int IN3 = 11;
int IN4 =10;
int ENB = 3;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 200;

void setup()
{
  Serial.begin(115200);
 // join I2C bus (I2Cdev library doesn't do this automatically)
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif

 mpu.initialize();

 devStatus = mpu.dmpInitialize();

 // supply your own gyro offsets here, scaled for min sensitivity
 mpu.setXGyroOffset(0);
 mpu.setYGyroOffset(-15);
 mpu.setZGyroOffset(29);
 mpu.setZAccelOffset(1530); // 1688 factory default for my test chip

 // make sure it worked (returns 0 if so)
 if (devStatus == 0)
 {
 // turn on the DMP, now that it's ready
 mpu.setDMPEnabled(true);

 // enable Arduino interrupt detection
 attachInterrupt(0, dmpDataReady, RISING);
 mpuIntStatus = mpu.getIntStatus();

 // set our DMP Ready flag so the main loop() function knows it's okay to use it
 dmpReady = true;

 // get expected DMP packet size for later comparison
 packetSize = mpu.dmpGetFIFOPacketSize();
 
 //setup PID
 pid.SetMode(AUTOMATIC);
 pid.SetSampleTime(10);
 pid.SetOutputLimits(-255, 255); 
 }
 else
 {
 // ERROR!
 // 1 = initial memory load failed
 // 2 = DMP configuration updates failed
 // (if it's going to break, usually the code will be 1)
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
  mySerial.begin(9600);
}


void loop()
{
 // if programming failed, don't try to do anything
 if (!dmpReady) return;

 // wait for MPU interrupt or extra packet(s) available
 while (!mpuInterrupt && fifoCount < packetSize)
 {
 //no mpu data - performing PID calculations and output to motors 
 pid.Compute();
 output_left = factor_left*output;
 output_right = factor_right*output;
 motorController.move(output_left,output_right,MIN_ABS_SPEED);
 
 }

 // reset interrupt flag and get INT_STATUS byte
 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();

 // get current FIFO count
 fifoCount = mpu.getFIFOCount();

 // check for overflow (this should never happen unless our code is too inefficient)
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 // reset so we can continue cleanly
 mpu.resetFIFO();
 Serial.println(F("FIFO overflow!"));

 // otherwise, check for DMP data ready interrupt (this should happen frequently)
 }
 else if (mpuIntStatus & 0x02)
 {
 // wait for correct available data length, should be a VERY short wait
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

 // read a packet from FIFO
 mpu.getFIFOBytes(fifoBuffer, packetSize);
 
 // track FIFO count here in case there is > 1 packet available
 // (this lets us immediately read more without waiting for an interrupt)
 fifoCount -= packetSize;

 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180/M_PI + 180;
 }
mySerial_control();
unsigned long currentTime = millis();
if (currentTime - lastPrintTime >= printInterval) {
 lastPrintTime = currentTime;
 mySerial.println(ypr[1] * 180/M_PI + 180-originalSetpoint);
 //mySerial.println(fifoCount);
 //mySerial.println((mpuIntStatus & 0x10));
 //Serial.print(Kp);
 //Serial.print(Kd);
 //Serial.println(Ki);
 Serial.println(output);
 Serial.println(output_left);
 Serial.println(output_right);
 Serial.println(content);
}
}
void mySerial_control() {
  double d_speed=3;
  if(mySerial.available()){
     content1=mySerial.read();
     if(content1=='\n'){
       content="";
     }else {
     content += content1;
     }
     if(content[0]=='F')
     {
     setpoint = originalSetpoint - d_speed;    
     }      
     else if(content[0]=='B')
     {
     setpoint = originalSetpoint + d_speed;
     }
     else if(content[0]=='L')
     {
     setpoint = originalSetpoint - d_speed;
     factor_left=-1;
     factor_right=1;
     }
     else if(content[0]=='R')
     {
     setpoint = originalSetpoint - d_speed;
     factor_right=1;
     factor_left=-1;
     }
     else if(content[0]=='S')
     {
       setpoint= originalSetpoint;
       motorSpeedFactorRight = originalMotorSpeedFactorRight;
       motorSpeedFactorLeft = originalMotorSpeedFactorLeft;
       factor_left=1;
       factor_right=1;
     }
     else if(content[0]=='K')
     {
      double number = extractNumber(content);
       if(content[1]=='P')
       {
          Kp = number;
       }
       else if(content[1]=='D')
       {
          Kd = number;
       }
       else if(content[1]=='I')
       {
          Ki = number;
       }
     }
     if(content[0]=='V')
     {
       double number = extractNumber(content);
        if(content[1]=='R')
        {
          originalMotorSpeedFactorLeft = number;
        }
        else if(content[1]=='L')
        {
          originalMotorSpeedFactorRight = number;
        }
     }
  } 
}
double extractNumber(String str) {
  for (int i = 0; i < str.length(); i++) {
    if (isDigit(str.charAt(i))) {
      return str.substring(i).toDouble();
    }
  }
}