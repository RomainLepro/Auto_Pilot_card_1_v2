
#ifndef AVION_H
#define AVION_H
#include "Arduino.h"
#include "Servo.h"
#include "STAB.h"
#include "PID.h"
#include <PinChangeInt.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

bool environ(int a,int b,int c = 50){
  return abs(a-b)<c;
}

class IMU{
    static bool mpuInterrupt; 
    static void dmpDataReady() {mpuInterrupt = true;}
    public:
    #define serial_com true
    #define OUTPUT_READABLE_YAWPITCHROLL 
    #define LED_PIN 13
    MPU6050 mpu;
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
    // ================================================================
    // ===               INTERRUPT DETECTION ROUTINE                ===
    // ================================================================
    void init(){
        mpuInterrupt = false;
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            Wire.begin();
            TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
        #endif
        if(serial_com){Serial.println(F("Initializing I2C devices..."));}
        mpu.initialize();
        if(serial_com){Serial.println(F("Testing device connections..."));
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        Serial.println(F("Initializing DMP..."));}
        devStatus = mpu.dmpInitialize();
        if (devStatus == 0) {
            mpu.CalibrateAccel(8);
            mpu.CalibrateGyro(8);
            mpu.PrintActiveOffsets();
            //Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);
            //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
            attachInterrupt(0, dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();
            //Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            //Serial.print(F("DMP Initialization failed (code "));
            //Serial.print(devStatus);
            //Serial.println(F(")"));
        }
        pinMode(LED_PIN, OUTPUT);
    }
    void getAngles(float *Ox,float *Oy,float *Oz){
        *Ox = ypr[2]*-180/M_PI;
        *Oy = ypr[1]*180/M_PI;
        *Oz = ypr[0]*-180/M_PI;
    };

    void run(){
        if (!dmpReady) return;
    }
    void end(){
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
};

class Autopilot{
    public:
    float Oxgoal,Oygoal,Ozgoal;
    float Ox,Oy,Oz;
    PID OServox,OServoy,OServoz;
    LOWPASS filter_radiox,filter_radioy,filter_radioz;
    PID_GROUP plane_pid;
    void init(){
        
    }
};

class Radio{
    public:
    #define nb_channel 9
    static u8 pin_radio; //the pin_radio of the receiver
    static uint32_t t;
    static uint8_t channel;
    static volatile uint16_t rc_shared[nb_channel];

    uint16_t rc_values[nb_channel];
    
    Radio(u8 pinrad = 8){
        pin_radio=pinrad;
    }
    void rc_read_values() {
        noInterrupts();
        memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
        interrupts();}
    static void pinFunct(){ 
        if(digitalRead(pin_radio)){
            rc_shared[channel] = uint16_t(micros()-t);
            t = micros();  
            channel++;}
        else{if (uint16_t(micros()-t)>5000){channel = 0;}}
        if(channel == nb_channel){channel = 0;}}
    void init(){
        pinMode(pin_radio,INPUT);
        digitalWrite(pin_radio, LOW);
        PCintPort::attachInterrupt(this->pin_radio,pinFunct, CHANGE);
        }
    void run(){
        rc_read_values();
    }
};

class Servos{
    public:
    uint8_t myServosLength = 6;
    uint8_t L_pin_servos[6] = {6,7,9,10,11,12};
    Servo  myservo0,myservo1,myservo2,myservo3,myservo4,myservo5;
    Servo myServos[6] = {myservo0,myservo1,myservo2,myservo3,myservo4,myservo5};
    //could be mor motor, or should define an object :=> motor or servo
    uint8_t myMotorsLength = 2;
    uint8_t L_pin_motors[2] = {4,5};
    Servo  mymotor0,mymotor1;
    Servo myMotors[2] = {mymotor0,mymotor1};
    Servos(){
    };
    void init(){
        for (int i=0;i<=myServosLength;i++){
            myServos[i].attach(L_pin_servos[i]);
            myServos[i].write(90); //to calm servos down
        }
        for (int i=0;i<=myMotorsLength;i++){
            myMotors[i].attach(L_pin_servos[i]);
            myMotors[i].write(0); //to calm motors down
        }
        shake(8); //shake 8 time after start
    };
    void shake(int k = 4){
        for (int i=0; i<=k; i++){
            for (int j =0;j<myServosLength;j++){
                myServos[j].write(90+((-1)^(i))*25);
                delay(50);
            }
            for(int j=0;j<myServosLength;j++){
                myServos[j].write(90);
                delay(50);  
            }
        }
    };
    void updateServos(int valServos[6],int valMotors[2]){
        
    }
};

class Com{
    public:
    float dst,cor,hom,dOz,dri,alt,dal,vit;
    #define SERIAL_PORT_SPEED 115200
    String msg;
    Com(){
    };
    void init(){
        Serial.begin(SERIAL_PORT_SPEED);
    };
    void run(){
        while(Serial.available()>0){
        msg = Serial.readStringUntil('\n');
        if(msg != ""){//read serial data send by the other card
            if (msg.startsWith("dst: ")){dst = msg.substring(5).toFloat();}
            else if (msg.startsWith("cor: ")){cor = msg.substring(5).toFloat();}
            else if (msg.startsWith("hom: ")){hom = msg.substring(5).toFloat();}
            else if (msg.startsWith("dOz: ")){dOz = msg.substring(5).toFloat();}//each time there's a new comand sended, save the current value of Oz
            else if (msg.startsWith("dri: ")){dri = msg.substring(5).toFloat();}
            else if (msg.startsWith("alt: ")){alt = msg.substring(5).toFloat();}
            else if (msg.startsWith("dal: ")){dal = msg.substring(5).toFloat();}
            else if (msg.startsWith("vit: ")){vit = msg.substring(5).toFloat();}
            else if (msg.startsWith("wait: ")){cor =0; alt = 0;}}}//if no signal, go straight
        msg="";
        //Serial.print("dét: ");Serial.print(dst);Serial.print(" -alt: ");Serial.print(alt);Serial.print(" -cor: ");Serial.print(cor);Serial.print(" -hom: ");Serial.println(hom);
    }
};


enum FlightMode {canard,vtail,vtol};
enum ControlMode {mode0,mode1,mode2};

class Plane {
    public:
    FlightMode flightmode = canard;
    ControlMode controlmode = mode1;
    float Ox,Oy,Oz;
    float Ogoalx,Ogoaly,Ogoalz,ThrotleComande; 
    float Wx,Wy,Wz;
    float x,y,z;
    float Vx,Xy,Vz;
    float radio_control;
    u16 time;
    IMU imu;
    Autopilot autopilot; //lois de comande de l'avion
    Radio radio; 
    Servos servos;
    Com com; //hiden Serial
    uint16_t dt;
    uint32_t t,previous_t;
    void init(){
        imu.init();
        autopilot.init();
        radio.init();
        servos.init();
        com.init();
    }
    void test(){
    }
    void run(){
        dt = millis()-t;
        t = millis();
        imu.getAngles(&Ox,&Oy,&Oz);
        Ogoalx = degres(radio.rc_shared[1]);Ogoaly = degres(radio.rc_shared[2]);Ogoalz = degres(radio.rc_shared[4]);ThrotleComande = degres(radio.rc_shared[3]);

        if(environ(radio.rc_shared[5],2000)){controlmode = mode2;}// retour maison 
        else if(environ(radio.rc_shared[5],1000)){controlmode = mode0;}//rc
        else{controlmode = mode1;}//defaut : RC stab

        if(controlmode == 0){radio_control =1;}
        else if(controlmode == 1){radio_control =0.5;}
        else{radio_control =0;}
        autopilot.plane_pid.update(t,Ox,Oy,Oz,Ogoalx,Ogoaly,Ogoalz,ThrotleComande,radio_control);
    }
};
#endif