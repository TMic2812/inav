#include <stdint.h>
#include <math.h>
#include <platform.h>

#include "common/maths.h"
#include "navigation.h"
#include "navigation/atkmode.h"
#include "common/axis.h"

static float initialx;
static float initialy;
//read in goal relative to atkhome
static float goalx = 10;
static float goaly = 10;



void saveAtkStartPosition(t_fp_vector * pos){
  //reads in home position for this mode.
    initialx = pos->V.X;
    initialy = pos->V.Y;
}

void setAtkDestination(t_fp_vector * pos){
   // static float previousTime;

   // if ((time - previousTime) > 100){
     //   goalx += 0.1;
     //sets position to initialised position + goal position
        pos->V.X = initialx + goalx;
        pos->V.Y = initialy + goaly;
  //  }

}