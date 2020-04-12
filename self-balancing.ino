#include <StringSplitter.h>

#include "Wire.h"
#include <PID_v1.h>
#include <MotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define SPEED_FACTOR 1

double motor1SpeedFactor = 1;
double motor2SpeedFactor = 1;

MPU6050 sensor;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 181.5;
double setpoint = originalSetpoint;
double movingAngleOffset = 0;
double input, output;

//adjust these values to fit your own design
double Kp = 10;   
double Kd = 0;
double Ki = 0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}

/*
 L9110 motor driver controlando 2 motores CC
 COPIAR Y PEGAR SOBRE EL SKETCH DE ARDUINO
*/

const int AIA = 5;  // (pwm) pin 6 conectado a pin A-IA 
const int AIB = 6;  // (pwm) pin 5 conectado a pin A-IB 
const int BIA = 9; // (pwm) pin 10 conectado a pin B-IA  
const int BIB = 10;  // (pwm) pin 9 conectado a pin B-IB
 
//MOTOR CONTROLLER
MotorController motorController1(AIA, AIB, false);
MotorController motorController2(BIA, BIB, false);
  
// cambie este valor (100 a 255) para controlar 
// la velocidad de los motores 

void setup() {
  initializeIMU();
}

void initializeIMU() {
  Serial.begin(57600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  calibrateOffset();
  
  devStatus = sensor.dmpInitialize();
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    sensor.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = sensor.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = sensor.dmpGetFIFOPacketSize();

    //setup PID
    initializePID();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void calibrateOffset() {
  // CALIBRACION ACOSTADO: -1826 2038  1026  -60 -15 -70
  // CALIBRACION PARADO: -3727  2081  2622  -60 -15 -81

 
  sensor.setXAccelOffset(-1826);
  sensor.setYAccelOffset(2038);
  sensor.setZAccelOffset(1026);

  sensor.setXGyroOffset(-60);
  sensor.setYGyroOffset(-15);
  sensor.setZGyroOffset(-70);

 /*
  sensor.setXAccelOffset(-3727);
  sensor.setYAccelOffset(2081);
  sensor.setZAccelOffset(2622);

  sensor.setXGyroOffset(-60);
  sensor.setYGyroOffset(-15);
  sensor.setZGyroOffset(-81);
  */
}

void initializePID() {
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);   
}

void loop() {  
  if (Serial.available()) {
    String newConfig = Serial.readString();

    StringSplitter *splitter = new StringSplitter(newConfig, ',', 4);
    double newKp = splitter->getItemAtIndex(0).toDouble();
    double newKd = splitter->getItemAtIndex(1).toDouble();
    double newKi = splitter->getItemAtIndex(2).toDouble();
    double newMovingAngleOffset = splitter->getItemAtIndex(3).toDouble();

    pid.SetTunings(newKp, newKd, newKi);
    movingAngleOffset = newMovingAngleOffset;

  }
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    double speed = -output/255.0;

    Serial.print(speed);Serial.print('\t');
    Serial.println(input);

    motorController1.set(motor1SpeedFactor * SPEED_FACTOR * speed);
    motorController2.set(motor1SpeedFactor * SPEED_FACTOR * speed);
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = sensor.getIntStatus();

  // get current FIFO count
  fifoCount = sensor.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    sensor.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = sensor.getFIFOCount();
    
    // read a packet from FIFO
    sensor.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    sensor.dmpGetQuaternion(&q, fifoBuffer);
    sensor.dmpGetGravity(&gravity, &q);
    sensor.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    double yaw = ypr[0] * 180/M_PI + 180;
    double pitch = ypr[1] * 180/M_PI + 180;
    double roll = ypr[2] * 180/M_PI + 180;

    input = pitch + movingAngleOffset;
  }
}
