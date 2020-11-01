#include <Arduino.h>

//Aircraft Stabilizer
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include <Servo.h>
#include <PinChangeInt.h>
#include "PID.h"

#define SERIAL_PORT_SPEED 115200
// ================================================================
// ===                      RECIEVER                            ===
// ================================================================
# define pin 8
# define nb_channel 9

#define Vtail true
#define Test true

String msg;

uint32_t t;
uint8_t channel;
uint16_t rc_values[nb_channel];
volatile uint16_t rc_shared[nb_channel];

void pinFunct(){ 
  if(digitalRead(pin)){
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
int OComandex = 0,OComandey = 0,OComandez = 0,ThrotleComande = 0; 
int OServox = 0,OServoy = 0,OServoz = 0,ThrotleServo = 0; 

uint16_t dt1;
uint32_t t1;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {mpuInterrupt = true;}
// ================================================================
// ===                         ms to deg                        ===
// ================================================================
int degres(int t){
  return (t-1500)*18/100;}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

int sign(int a){
  if (a<0)return -1;
  return 1;
}

#define roll 9
#define pitch 10
#define yaw 12
#define throtle 11

float offsetx = 0;
float offsety = 0;
float offsetz = 0;
float offsetthrottle = 0;
uint32_t timer_mode_3=0, timer_mode_4=0, timer_mode_5=0 ,timer_mode_2=0;


Servo myservoRoll; 
Servo myservoPitch; 
Servo myservoYaw; 
Servo myservoThrotle; 

uint8_t mode = 0;

void shake(int k = 4){
  for (int i=1; i<=k; i++){
    myservoPitch.write(90+(-1)^(i)*25);
    myservoRoll.write(90+(-1)^(i)*25);
    myservoYaw.write(90+(-1)^(i)*25);delay(150);}
  myservoPitch.write(90);
  myservoRoll.write(90);
  myservoYaw.write(90);
}

float ssqrt(float a,int b=100){if (a>0){return min(sqrt(a),b);}else {return -min(sqrt(-a),b);}}

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  
  pinMode(pin,INPUT);
  digitalWrite(pin, LOW);
  PCintPort::attachInterrupt(pin,pinFunct, CHANGE);
  
  myservoPitch.attach(pitch);
  myservoRoll.attach(roll);
  myservoYaw.attach(yaw);
  myservoThrotle.attach(throtle);
  
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.CalibrateAccel(8);
        mpu.CalibrateGyro(8);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(LED_PIN, OUTPUT);

    shake(8); //shake 8 time after start
}

// ================================================================

void loop() {
  if (!dmpReady) return;

  int dst,cor,hom,alt,vit;
    while(Serial.available()>0){
      msg = Serial.readStringUntil('\n');
      if(msg != ""){//read serial data send by the other card
        if (msg.startsWith("dst: ")){dst = msg.substring(5).toFloat();}
        else if (msg.startsWith("cor: ")){cor = msg.substring(5).toFloat();}
        else if (msg.startsWith("hom: ")){hom = msg.substring(5).toFloat();}
        else if (msg.startsWith("alt: ")){alt = msg.substring(5).toFloat();}
        else if (msg.startsWith("vit: ")){vit = msg.substring(5).toFloat();}
        else if (msg.startsWith("wait: ")){cor =0; alt = 0;}}}//if no signal, go straight
    msg="";
    //Serial.print("dét: ");Serial.print(dst);Serial.print(" -alt: ");Serial.print(alt);Serial.print(" -cor: ");Serial.print(cor);Serial.print(" -hom: ");Serial.println(hom);
    rc_read_values();
    uint16_t summ = 0;
    Ox = ypr[2]*-180/M_PI;Oy = ypr[1]*180/M_PI;Oz = ypr[0]*-180/M_PI;    
    dt1 = millis()-t1;
    t1 = millis();

    int a=0;int b=0;int c=0;int d=0;
    //mode0 = normal; mode1 = stabalized; mode2 = takeof
    c=degres(rc_shared[6])/2;//trimage vol horrizontal
    if(environ(rc_shared[5],2000)){if(mode < 2){mode = 2;}a=5;b=0.5;}// retour maison 
    else if(environ(rc_shared[5],1500)){mode = 1;a=3;b=1;}//Stabilisé : asciete réglable 
    else{mode = 0;a=0;b=1;}//defaut : RC
    OServox = (Ox+offsetx/40)*a- degres(rc_shared[1])*b ;
    OServoy = (Oy-c-d+offsety/40)*a-degres(rc_shared[2])*b;
    OServoz = (offsetz/40)*a + degres(rc_shared[4])*b; 
    //si retour maison: retour maison actif si on est trop loin
    //passage au mode 3 (trim) si buté en haut a droite joystick droit
    if (mode>=2){
      OServoz = ssqrt(hom,8)*10;OServox += ssqrt(hom,5)*5;OServoy += ssqrt(alt,8)*5;

      if(environ(degres(rc_shared[1]),-90,25) && environ(degres(rc_shared[2]),-90,25) ){
        if(timer_mode_3==0){timer_mode_3 = millis(); }
        else if(millis()-timer_mode_3>500){ mode = 3;timer_mode_3 = 0;shake(3);}
      }
      else if(environ(degres(rc_shared[1]),-90,25) && environ(degres(rc_shared[2]),90,25) ){
        if(timer_mode_4==0){timer_mode_4 = millis(); }
        else if(millis()-timer_mode_4>500){ mode = 4;timer_mode_4 = 0;shake(4);}
      }
      else if(environ(degres(rc_shared[1]),90,25) && environ(degres(rc_shared[2]),90,25) ){
        if(timer_mode_5==0){timer_mode_5 = millis(); }
        else if(millis()-timer_mode_5>500){ mode = 5;timer_mode_5 = 0;shake(5);}
      }
      else if(environ(degres(rc_shared[1]),90,25) && environ(degres(rc_shared[2]),-90,25) ){
        if(timer_mode_2==0){timer_mode_2 = millis(); }
        else if(millis()-timer_mode_2>500){ mode = 2;timer_mode_2 = 0;shake(2);}
      }
      else{
        if (mode == 3){
          offsetx += (degres(rc_shared[1])/45);
          offsety += (degres(rc_shared[2])/45);
          offsetz += (degres(rc_shared[4])/45);
          offsetx = max(min(offsetx,500),-500);
          offsety = max(min(offsety,500),-500);
          offsetz = max(min(offsetz,500),-500);
          }
        else if(mode==4){
          offsetx = 0;offsety = 0;offsetz = 0;offsetthrottle = 0;
        }
        else{
          timer_mode_3=0;timer_mode_4=0;timer_mode_5=0;timer_mode_2=0;}
      }
    }

    ThrotleServo = degres(rc_shared[3]) + offsetthrottle;
    ThrotleServo = max(-55,ThrotleServo);
    ThrotleServo = min(75,ThrotleServo);
     Serial.print("OX : ");Serial.print(OServox);Serial.print(" POTATO   ");
    Serial.print("OX : ");Serial.print(OServox);Serial.print("    ");
    //Serial.print("mode : ");Serial.print(mode);Serial.print("    ");
    Serial.print("OY : ");Serial.print(OServoy);Serial.print("    ");
    Serial.print("OZ : ");Serial.print(OServoz);Serial.print("    ");Serial.println("");
    //Serial.print("Throttle : ");Serial.print(ThrotleServo);Serial.print("    ");
    //if(timer_mode==0){Serial.print(timer_mode);Serial.print("    ");}
    //else{Serial.print(millis()-timer_mode);Serial.print("    ");}
    //Serial.print("throttle");Serial.print(degres(rc_shared[3]) + offsetthrottle);Serial.print("    ");
    //Serial.println("");
    //-57-60 = bip  begin = -40 stop 85 trim du throttle, étrange mais necessaire ....
    
    if (Vtail){    
      myservoRoll.write(-OServox+90); 
      myservoPitch.write((OServoz-OServoy)/2+90); 
      myservoYaw.write((OServoy+OServoz)/2+90);
      myservoThrotle.write(ThrotleServo+90);
      }

    else{
      myservoRoll.write(OServox+90); 
      myservoPitch.write(OServoy+90); 
      myservoYaw.write(OServoz+90);
      myservoThrotle.write(ThrotleServo+90);} 

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    } 
}