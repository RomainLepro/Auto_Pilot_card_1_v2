/**   Ensmasteel Library - LIDAR management
 * note : Exploitation of LIDAR's data : collision avoidance, trajectory optimisation
 * author : EnsmaSteel
 * date : November 2019
*/

//points to check :
//use


#ifndef STAB_H
#define STAB_H

#include "Arduino.h"

// ================================================================
// ===                         ms to deg                        ===
// ================================================================
float degres(int t){return float(t-1500)*18.0/100;}

int sign(int a){
  if (a<0){return -1;}
  return 1;
}

float ssqrt(float a,int b=100){if (a>0){return min(sqrt(a),b);}else {return -min(sqrt(-a),b);}}

#endif
