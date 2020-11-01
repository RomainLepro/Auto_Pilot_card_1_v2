#include "PID.h"
#include "Arduino.h"
#include "MODULO.h"




float bound(float val,float bound){//put boundarry on values
    if (val>bound){val = bound;}
    else if(val<-bound){val = -bound;}
    return val;
}


bool PID::update(float UE,uint16_t time_millis)
{
    #define a 0.85
    #define b 0.5
    
    float dt = float(time_millis-prev_t)/1000;
   if (prev_t!=0 && dt!=0){   
       I += UE*dt*GI;
       I = bound(I,MAX_I);
       P = (b*P+(1-b)*GP*UE);
       P = bound(P,MAX_P);
       if(UE != prev_UE && time_millis-prev_t_D>2){
           D = bound((a*D + GD*(1-a)*(UE-prev_UE)/float(time_millis-prev_t_D)*1000),MAX_D);
           prev_t_D = time_millis;
           prev_UE = UE;}
       US = bound(G*(P+I+D),MO);
       prev_t = time_millis;
       return true;
   }
   else{
       prev_t = time_millis;
       US =bound(G*P*GP,MO);
       return false;}}

bool LOWPASS::update(float UE,uint16_t time_millis)
{
    if(angle){
        if(US-UE>180){UE+=360;}//UE is negativ an US positive => put UE positive
        else if(US-UE<-180){UE-=360;}//US is negativ an UE positive => put UE negative
        }
    US = UE*taux + US*(1-taux);
    if(angle){US = modulo(US);}
    //prev_UE = UE;
    //prev_prev_UE = prev_UE;
    return true;
}


PID_GROUP::PID_GROUP(){
    filter_Ox.taux = 0.6;
    filter_Oy.taux = 0.6;
    filter_Oz.taux = 0.6;
    filter_Ogoalz.taux = 0.002;
    filter_Ogoaly.taux = 0.002;
    filter_Ogoalz.angle = true;
};


bool PID_GROUP::update(uint32_t time,float Ox,float Oy,float Oz,float Ogoalx,float Ogoaly,float Ogoalz,float Throtle_goal,float radio_control)
{
    #define min_max 35//minmax for servo
    float stab_control = 1-radio_control;

    //filtrage input data
    filter_Ogoalx.update(Ogoaly,time);filter_Ogoaly.update(Ogoaly,time);filter_Ogoalz.update(Ogoalz,time);
    filter_throtle_goal.update(Throtle_goal,time);
    filter_Ox.update(Ox,time);filter_Oy.update(Oy,time);filter_Oz.update(Oz,time);
    
    // PID
    PIDX.update(-filter_Ox.US*stab_control+filter_Ogoalx.US*radio_control,time);
    PIDY.update((filter_Oy.US+filter_Ogoaly.US)*stab_control-filter_Ogoaly.US*radio_control,time);
    PIDZ.update(modulo(filter_Oz.US-filter_Ogoalz.US)*stab_control+filter_Ogoalz.US*radio_control,time);
    ThrotleServo = filter_throtle_goal.US;
    ThrotleServo *= 0.5; // remap of the throtle
    //min(33,max(-33,OServoz.US))
    OX = PIDX.US;OY = PIDY.US;OZ = PIDZ.US;
    return true;
}