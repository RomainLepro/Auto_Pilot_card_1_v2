/**   Ensmasteel Library - LIDAR management
 * note : Exploitation of LIDAR's data : collision avoidance, trajectory optimisation
 * author : EnsmaSteel
 * date : November 2019
*/

//points to check :
//use


#ifndef PID_H
#define PID_H

#include "Arduino.h"

const float MAX_O = 90; //maximum output of the PID
const float MAX_P = 60; //Maximum gain taken by the proportionnal portion of the PID
const float MAX_I = 20;
const float MAX_D = 40;

const float GAIN_P = 3; //Default Gain of the proportionnal portion of the PID
const float GAIN_I = 0.5;
const float GAIN_D = 0.5;
const float GAIN = 1;


class LOWPASS
{
public:
    bool angle = false;
    float taux = 0.04;
    float US = 0,prev_US = 0,prev_UE = 0;float prev_prev_UE=0;
    bool update(float UE,uint16_t time_millis);
private:
    uint16_t prev_time_millis = 0;
};


class PID
{
public:
    LOWPASS FD;
    uint8_t dt = 0;
    float US = 0; //the current value of the output/input
    float G = GAIN;
    float MO = MAX_O;
    float GP = GAIN_P,GI = GAIN_I,GD = GAIN_D;
    float MP = MAX_P,MI = MAX_I,MD = MAX_D;
    float P=0,I = 0,D=0;
    //return true if prev_t !=0 && dt !=0 (= the PID was initialized and there's a new value)
    bool update(float UE,uint16_t time_millis);//each time a new value is added, calculate the new output
private:
    uint16_t prev_t = 0,prev_t_D = 0;
    float prev_P = 0;float prev_UE=0;
};


class PID_GROUP
{
public:
    PID PIDX,PIDY,PIDZ;
    LOWPASS filter_Ox,filter_Oy,filter_Oz;
    LOWPASS filter_Ogoalx,filter_Ogoalz,filter_Ogoaly,filter_throtle_goal;
    float OX,OY,OZ,ThrotleServo;
    PID_GROUP();
    bool update(uint32_t time,float Ox,float Oy,float Oz,float Ogoalx,float Ogoaly,float Ogoalz,float throtle_goal,float radio_control);
private:
};

#endif
