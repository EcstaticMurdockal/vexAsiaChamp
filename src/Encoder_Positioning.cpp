#include "vex.h"
#include "Move.h"
#include "PID.h"
#include <ctime>
#include "LPF.h"
using namespace vex;

double hrz = 0;
double vtc = 0;

double lastvx = 0;
double lastvy = 0;

const float wheel_vertical = 2.54;
const float wheel_horizontal = 2.54;
const float distance_vertical = 0;// 定位轮偏移量
const float distance_horizontal = 0;

float lastangle = 0;
float Last_position_r = 0;
float Last_position_s = 0;
/*
void Enc_pos(){

    /*hrz = Horizontal.velocity(rpm) * 3.14159 * 10.16 * 36 / 84 / 60;
    vtc = Vertical.velocity(rpm) * 3.14159 * 5.08 / 60;

    X -= dt/1000 * (cos((90 - Inertial.rotation())/180*3.14) * vy + cos((180 - Inertial.rotation())/180*3.14) * vx); // Integrating ax, ay under relative coordinate system into vx, vy under absolute(starting) coordinate system
    Y += dt/1000 * (sin((90 - Inertial.rotation())/180*3.14) * vy + sin((180 - Inertial.rotation())/180*3.14) * vx);

    //sgnAx = sgn(vx - lastvx);
    //sgnAy = sgn(vy - lastvy);
    lastvx = vx;
    lastvy = vy;
    *
      double R = (Vertical.position(deg) - Last_position_r) * (3.1416/180) * wheel_vertical; 
      double S = (Horizontal.position(deg) - Last_position_s) * (3.1416/180) * wheel_horizontal;
      double h; 
      double i;
      double h2;
      double degr = Inertial.rotation(deg);
      double a = (degr*3.1416/180) - lastangle; 
      if (fabs(a) != 0){
          double r = R / a; 
          i = a / 2.0;
          double sinI = sin(i);
          h = ((r + distance_vertical) * sinI) * 2.0;
  
          double r2 = S / a; 
          h2 = ((r2 + distance_horizontal) * sinI) * 2.0;
      }
      else{
          h = R;
          i = 0;
          h2 = S;
      }
      double p = i + lastangle; 
      double cosP = cos(p);
      double sinP = sin(p);
  
      Y += h * cosP;
      X += h * sinP;
  
      Y += h2 * -sinP; 
      X += h2 * cosP; 
  
    lastangle = (degr*3.1416/180);
    Last_position_r = Vertical.position(deg);
    Last_position_s = Horizontal.position(deg);
}
*/
double x_l = 0;
double y_l = 0;
double x_r = 0;
double y_r = 0;

double l_enc = 0;
double r_enc = 0;
double last_l_enc = 0;
double last_r_enc = 0;

void Pos_Motor(){
    l_enc = (lm1_enc + lm2_enc + lm3_enc) / 3;
    r_enc = (rm1_enc + rm2_enc + rm3_enc) / 3;

    x_l += (l_enc - last_l_enc) * sin(H);
    y_l += (l_enc - last_l_enc) * cos(H);

    x_r += (r_enc - last_r_enc) * sin(H);
    y_r += (r_enc - last_r_enc) * cos(H);

    X = (x_l + x_r) / 2;
    Y = (y_l + y_r) / 2;

    X = X * 8.255 / 600;
    Y = Y * 8.255 / 600;

    last_l_enc = l_enc;
    last_r_enc = r_enc;
}






