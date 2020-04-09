// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;


/*
 L9110 motor driver controlando 2 motores CC
 COPIAR Y PEGAR SOBRE EL SKETCH DE ARDUINO
*/

const int AIA = 5;  // (pwm) pin 6 conectado a pin A-IA 
const int AIB = 6;  // (pwm) pin 5 conectado a pin A-IB 
const int BIA = 9; // (pwm) pin 10 conectado a pin B-IA  
const int BIB = 10;  // (pwm) pin 9 conectado a pin B-IB 

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
  
}

void initializeMotors() {
  pinMode(AIA, OUTPUT); // fijar los pines como salidas
  pinMode(AIB, OUTPUT);
  pinMode(BIA, OUTPUT);
  pinMode(BIB, OUTPUT);
  
}

void loop() {
  //ejecutarMovimientos();
  transmitirDatos();
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
