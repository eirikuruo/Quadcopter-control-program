/* This program accepts serial input and updates PID controller variables
* accordingly. The PID controllers get data from a gyroscopic sensor and 
* calculate motor control output.
*/
#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

//*************************************************************
//Initial Setpoint for PID regulators--------------------------
const int YawInitialSetpoint=0;
const int PitchInitialSetpoint=0;
const int RollInitialSetpoint=0;
//ESC configuration--------------------------------------------
const int ESC_MIN = 22;
const int ESC_MAX = 115;
const int ESC_TAKEOFF_OFFSET = 30;
const int ESC_ARM_DELAY = 5000;
//Initial PID configuration------------------------------------
//Pitch controller
const int PITCH_P_VAL = 0.5;
const int PITCH_I_VAL = 0;
const int PITCH_D_VAL = 1;
//Roll controller
const int ROLL_P_VAL = 2;
const int ROLL_I_VAL = 5;
const int ROLL_D_VAL = 1;
//Yaw controller
const int YAW_P_VAL = 2;
const int YAW_I_VAL = 5;
const int YAW_D_VAL = 1;
//Flight parameters---------------------------------------------
const int PITCH_MIN = -30;
const int PITCH_MAX = 30;
const int ROLL_MIN = -30;
const int ROLL_MAX = 30;
const int YAW_MIN = -180;
const int YAW_MAX = 180;
const int PID_PITCH_INFLUENCE = 20;
const int PID_ROLL_INFLUENCE = 20;
const int PID_YAW_INFLUENCE = 20;
//Arduino Pin configuration------------------------------------
const int motor1 = 2; // the pin that the motor1 is attached to
const int motor2 = 3; // the pin that the motor2 is attached to
const int motor3 = 4; // the pin that the motor3 is attached to
const int motor4 = 5; // the pin that the motor4 is attached to
//*************************************************************
//MPU variables-------------------------------------------------
//MPU interface object
MPU6050 mpu;  
//MPU control variables
uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 
//MPU data variables
Quaternion q;           // [w, x, y, z]         quaternion for mpu output
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};
//interrupt flag
volatile bool mpuInterrupt = false;
//Motor control variables---------------------------------------------------------------
//PID setpoint variables
float YawCon,PitchCon,RollCon,VelCon;
float YawConLast, PitchConLast, RollConLast, velocityLast;
//global velocity
int velocity;
//PID output variables
float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd
//motor velocities
int va, vb, vc, vd;
//axis balance velocities
int v_ac, v_bd;
//Servo objects, it is possible to use up to 8 servo objects see Arduino Servo library 
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
//PID objects-------------------------------------------------------------------------------
PID pitchReg(&ypr[1], &bal_bd, &PitchCon, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &bal_ac, &RollCon, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&ypr[0], &bal_axes, &YawCon, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);
//Serial variables--------------------------------------------------------------------------
int incomingByte;            //this variable is used for keeping data read from the serial buffer
unsigned long time,timelast; //variables for keeping data read from the internal timer
int countSignal;             //variable used as a counter for timeout from the serial connection
boolean check;               //variable used to check if the serial connection is active
//--------------------------------------------------------------------------------------------
//Setup function------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
void setup(){
  initSetpoint();
  initMPU();
  initESCs();
  initBalancing();
  initRegulators();
  timelast=50000;
  countSignal=0;
  Serial.begin(9600);
}
//---------------------------------------------------------------------------------------------
//loop function--------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
void loop(){
  //This loop runs while there is nothing in the MPU-6050 fifobuffer
  while(!mpuInterrupt && fifoCount < packetSize){
     
    /* Do nothing while MPU is not working
     * This should be a VERY short period
     */
      
  }
  //This code checks for serial commands every 200 milliseconds, if there is no response for 200 checks it decreases the global velocity by a set amount until it reaches zero
  //every time there is a serial command the check counter is reset.
  time = millis();
  if(time>timelast+200){
    check=SerialControl();
    if(check==false)
      countSignal++;
    else
      countSignal=0;
    timelast=time;
  }
  Check(countSignal);
  //This function gets information from the gyroscopic/accelerometer sensor
  getYPR();
  //These functions compare the information from the gyroscopic sensor with the setpoints in the PID controller
  //and adjust the output of each motor to maintain stability
  computePID();
  calculateVelocities();
  updateMotors();
  
}

/*  Check function
 *
 *  Checks if the input variable is greater than 200
 *  and the velocity control variable value is greater than 
 *  the minimum value. If both conditions are met 
 *  the velocity control variable value is decremented by 10.
 *  
 */
 void Check(int check){
   if(check>200&& VelCon > ESC_MIN)
     VelCon= VelCon-10;
 }

/*  SerialControl function
 *
 *  Checks for serial data, updates control variables,
 *  and sends sensor values if serial information is available
 *  returns false if no serial data is available.
 *
 */

boolean SerialControl(){
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital A
    if (incomingByte == 'A' && VelCon < ESC_MAX) {
        VelCon=VelCon+10;
    } 
    if (incomingByte == 'B' && VelCon > ESC_MIN) {
        VelCon=VelCon-10;
    }
    if (incomingByte == 'C' && PitchCon < PITCH_MAX) {
        PitchCon++;
    } 
    if (incomingByte == 'D' && PitchCon > PITCH_MIN) {
        PitchCon--;
    }
    if (incomingByte == 'E' && RollCon < ROLL_MAX) {
        RollCon++;
    } 
    if (incomingByte == 'F' && RollCon > ROLL_MIN) {
        RollCon--;
    }
    if (incomingByte == 'G' && YawCon < YAW_MAX) {
        YawCon++;
    } 
    if (incomingByte == 'H' && YawCon > YAW_MIN) {
        YawCon--;
    }
    // send sensor values:
    Serial.print((int)(aaWorld.x));
    Serial.print(",");
    Serial.print((int)(aaWorld.y));
    Serial.print(",");
    Serial.println((int)(aaWorld.z));
    return true;
  }
  else
    return false;
} 

/*  computePID function
 *
 *  formats data for use with PIDLib
 *  and computes PID output
 */

void computePID(){
  
  if((PitchCon < PITCH_MIN) || (PitchCon > PITCH_MAX)) PitchCon = PitchConLast;
  if((RollCon < ROLL_MIN) || (RollCon > ROLL_MAX)) RollCon = RollConLast;
  if((YawCon < YAW_MIN) || (YawCon > YAW_MAX)) YawCon = YawConLast;
  
  PitchConLast = PitchCon;
  RollConLast = RollCon;
  YawConLast = YawCon;
  
  ypr[0] = ypr[0] * 180/M_PI;
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;
  
  if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
  if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
  if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];
  
  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];

  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();

}

/*  getYPR function
 *
 *  gets data from MPU and
 *  computes pitch, roll, yaw on the MPU's DMP
 */

void getYPR(){
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024){ 
      
      mpu.resetFIFO(); 
    
    }else if(mpuIntStatus & 0x02){
    
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      fifoCount -= packetSize;
           
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    }

}

/*  calculateVelocities function
 *  
 *  calculates the velocities of every motor
 *  using the PID output
 */

void calculateVelocities(){

  velocity = VelCon;

  if((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;
  
  velocityLast = velocity;
  
  v_ac = (abs(-100+bal_axes)/100)*velocity;
  v_bd = ((100+bal_axes)/100)*velocity;
  
  va = ((100+bal_ac)/100)*v_ac;
  vb = ((100+bal_bd)/100)*v_bd;
  
  vc = (abs((-100+bal_ac)/100))*v_ac;
  vd = (abs((-100+bal_bd)/100))*v_bd;
  
  Serial.println(bal_bd);
  
  if(velocity < ESC_TAKEOFF_OFFSET){
  
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  
  }
  
}

void updateMotors(){

  servo1.write(va);
  servo2.write(vc);
  servo3.write(vb);
  servo4.write(vd);

}

void arm(){

  servo1.write(ESC_MIN);
  servo2.write(ESC_MIN);
  servo3.write(ESC_MIN);
  servo4.write(ESC_MIN);
  
  delay(ESC_ARM_DELAY);

}

void dmpDataReady() {
    mpuInterrupt = true;
}


void initMPU(){
  
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}

void initESCs(){

  servo1.attach(motor1);
  servo2.attach(motor2);
  servo3.attach(motor3);
  servo4.attach(motor4);
  
  delay(100);
  
  arm();

}

void initBalancing(){

  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;

}

void initRegulators(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}

void initSetpoint(){
  YawCon=YawInitialSetpoint;
  PitchCon=PitchInitialSetpoint;
  RollCon=RollInitialSetpoint;
  VelCon=ESC_MIN;
}
