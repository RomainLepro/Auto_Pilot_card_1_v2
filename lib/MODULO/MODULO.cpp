#include "MODULO.h"
#include "Arduino.h"

float modulo(float deg){
  float deg_corect = deg;
  int i = 0;
  while(abs(deg_corect)>=180 && i<10){
    if(deg_corect>=180){deg_corect-=360;}
    else if(deg_corect<=-180){deg_corect+=360;}i++;}
  return deg_corect;
}