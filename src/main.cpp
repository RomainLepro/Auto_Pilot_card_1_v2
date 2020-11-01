<<<<<<< HEAD

#include <Arduino.h>
=======
//#include <Arduino.h>
>>>>>>> 3cbed15d820e6dc7582977646359570bfa2c9b78
//Aircraft Stabilizer
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include <Servo.h>
#include <PinChangeInt.h>
#include "PID.h"
#include "STAB.h"
#include "MODULO.h"
<<<<<<< HEAD
#include "AVION.h"
=======
#define SERIAL_PORT_SPEED 115200
// ================================================================
// ===                      RECIEVER                            ===
// ================================================================
# define pin_radio 8 //the pin_radio of the receiver
# define nb_channel 9

//modes : Vtail-Vtol-Canard
#define flight_mode "Canard"

#define test_PID true
#define test_autopilot false
#define serial_com false
#define two_switch true


bool Vtail=true;
bool Vtol=false;
String msg;

uint32_t t; 
uint8_t channel;
uint16_t rc_values[nb_channel];
volatile uint16_t rc_shared[nb_channel];

float radio_control = 0;

void pinFunct(){ 
  if(digitalRead(pin_radio)){
    rc_shared[channel] = uint16_t(micros()-t);
    t = micros();  
    channel++;}
  else{if (uint16_t(micros()-t)>5000){channel = 0;}}
  if(channel == nb_channel){channel = 0;}}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();}

bool environ(int a,int b,int c = 50){
  return abs(a-b)<c;
}
// ================================================================
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//var used by receiver
#define RC_NUM_CHANNELS  6

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL 
#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//O of the plane
float Ox = 0,Oy = 0,Oz = 0;
float OComandex = 0,OComandey = 0,OComandez = 0,ThrotleComande = 0; 
float ThrotleServo = 0;

PID OServox,OServoy,OServoz;
LOWPASS filter_radiox,filter_radioy,filter_radioz;
PID_GROUP plane_pid;

float offsetx = 0;float offsety = 0;float offsetz = 0;float offsetthrottle = 0;

uint16_t dt1;
uint32_t t1,previous_t;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin_radio has gone high
void dmpDataReady() {mpuInterrupt = true;}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


uint32_t timer_mode_3=0, timer_mode_4=0, timer_mode_5=0 ,timer_mode_2=0;
>>>>>>> 3cbed15d820e6dc7582977646359570bfa2c9b78


// ================================================================
// ===                      RECIEVER                            ===
// ================================================================


void Canard (int OX, int OY,int OZ,int ThrotleServo,int valServos[6],int valMotors[2]){
  
  valServos[0]=OY+90;
  valServos[1]=OY+90;//pitch control
  valServos[2]=OY+90;
  valServos[3]=OX+90;
  valServos[4]=-OX+90;//roll control

  valMotors[0]=ThrotleServo+OZ+90;//throtle + yaw control
  valMotors[1]=ThrotleServo-OZ+90;
}




Plane avion;

void setup() {  
  avion.init();
}
// ================================================================
void loop() { 
   avion.run();
}


