#include "Wire.h"
#include <PID_v1.h>
#include <MotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define SPEED_FACTOR 0.8

double motor1SpeedFactor = 1;
double motor2SpeedFactor = 1;

MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

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
double originalSetpoint = 270;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 10;   
double Kd = 0;
double Ki = 0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
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

byte velocidad = 160;  
// cambie este valor (100 a 255) para controlar 
// la velocidad de los motores 

void setup() {
  initializeIMU();
  initializeMotors();
}

void initializeIMU() {
  Serial.begin(57600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

 
  sensor.setXAccelOffset(-1826);
  sensor.setYAccelOffset(2038);
  sensor.setZAccelOffset(1026);

  sensor.setXGyroOffset(-60);
  sensor.setYGyroOffset(-15);
  sensor.setZGyroOffset(-70);
 

  // CALIBRACION ACOSTADO: -1826 2038  1026  -60 -15 -70
  // CALIBRACION PARADO: -3727  2081  2622  -60 -15 -81

/*
  sensor.setXAccelOffset(-3727);
  sensor.setYAccelOffset(2081);
  sensor.setZAccelOffset(2622);

  sensor.setXGyroOffset(-60);
  sensor.setYGyroOffset(-15);
  sensor.setZGyroOffset(-81);
*/

  devStatus = sensor.dmpInitialize();
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
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
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255); 
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

void initializeMotors() {
  pinMode(AIA, OUTPUT); // fijar los pines como salidas
  pinMode(AIB, OUTPUT);
  pinMode(BIA, OUTPUT);
  pinMode(BIB, OUTPUT);
  
}

void loop() {
  //ejecutarMovimientos();
  //transmitirDatos();
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    motorController1.set(motor1SpeedFactor * SPEED_FACTOR * (output/255));
    motorController2.set(motor2SpeedFactor * SPEED_FACTOR * (output/255));
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = sensor.getIntStatus();

  // get current FIFO count
  fifoCount = sensor.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
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
    input = ypr[2] * 180/M_PI + 180;
  }
}

void leerSensor() {
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);  
  
}

void transmitirDatos() {
  leerSensor();
  //Mostrar las lecturas separadas por un [tab]
  Serial.print("a[x y z] g[x y z]:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  delay(100);

}

void ejecutarMovimientos() {
  izquierda();
  delay(1000);
  derecha();
  delay(1000);
  avanzar();
  delay(1000);
  retroceder();
  delay(1000);
  avanzar();
  delay(1000);
  
}

void avanzar()
{
  analogWrite(AIB, 0);
  analogWrite(AIA, velocidad);
  analogWrite(BIB, 0);
  analogWrite(BIA, velocidad);
}

void retroceder()
{
  analogWrite(AIB, velocidad);
  analogWrite(AIA, 0);
  analogWrite(BIB, velocidad);
  analogWrite(BIA, 0);
}

void izquierda()
{
  analogWrite(AIA, velocidad);
  analogWrite(AIB, 0);
  analogWrite(BIA, 0);
  analogWrite(BIB, velocidad);
}

void derecha()
{
  analogWrite(AIA, 0);
  analogWrite(AIB, velocidad);
  analogWrite(BIA, velocidad);
  analogWrite(BIB, 0);
}
