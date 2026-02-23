#include "cmath"
#include "vex.h"
#include "Move.h"
#include "LPF.h"
#include "PID.h"

void Move(int left_power, int right_power) {
  is_executing_movements = 1;
  if(left_power > 100){
    left_power = 100;
  }
  else if(left_power < -100){
    left_power = -100;
  }
  if(right_power > 100){
    right_power = 100;
  }else if(right_power < -100){
    right_power = -100;
  }
  
  Left_Motor1.spin(fwd, 0.128 * left_power, volt);
  Left_Motor2.spin(fwd, 0.128 * left_power, volt);
  Left_Motor3.spin(fwd, 0.128 * left_power, volt);

  Right_Motor1.spin(forward, 0.128 * right_power, volt);
  Right_Motor2.spin(forward, 0.128 * right_power, volt);
  Right_Motor3.spin(forward, 0.128 * right_power, volt);
}

void Move_Stop_brake(void) {
  is_executing_movements = 0;
  Left_Motor1.stop(brake);
  Left_Motor2.stop(brake);
  Left_Motor3.stop(brake);
  Right_Motor1.stop(brake);
  Right_Motor2.stop(brake);
  Right_Motor3.stop(brake);
}

void Move_Stop_coast(void) {
  is_executing_movements = 0;
  Left_Motor1.stop(coast);
  Left_Motor2.stop(coast);
  Left_Motor3.stop(coast);
  Right_Motor1.stop(coast);
  Right_Motor2.stop(coast);
  Right_Motor3.stop(coast);
}

void Move_Stop_hold(void) {
  is_executing_movements = 0;
  Left_Motor1.stop(hold);
  Left_Motor2.stop(hold);
  Left_Motor3.stop(hold);
  Right_Motor1.stop(hold);
  Right_Motor2.stop(hold);
  Right_Motor3.stop(hold);
}


void turnTo(double target,double pow){
  bool arrived = 0;

  while(!arrived){
    if(fabs(H -target) < 3){// Error allowed within 2 degrees
      arrived = 1;
    }

    if(H - target < 0){
      turn(pow, 0);
    }
    else {
      turn(-pow, 0);
    }

    task::sleep(20);
  }
  
  turn(0, 0); 
}

void toPos(double x, double y, double pow){
  //double ktp = 5.2;
  //double kti = 0.1;
  //double ktd = 0.5;

  double kturn = 1;
  double kturn0 = 2.7;//2.7

  const double kp = 10.6;//10.6
  const double ki = 0.029;//0.029
  const double kd = 126;//126

  bool arrived =0;

  double turnPower;
  double targetHeading = atan2(Y - y, X - x) * 180 / 3.14 + 90;;
  double output;
  double error = 0;
  double terror = 0;
  double last_error = error;
  double total_error = 0;
  double total_error_h = 0;
  double last_terror = error;
  double error0 = sqrt((X-x)*(X-x) + (Y-y)*(Y-y));
  double t = 0;

// Movements start here
//turnTo(targetHeading,100);

  while(!arrived){
    t += 10;

    targetHeading = atan2(Y - y, X - x) * 180 / 3.14 + 90; // Solution given by ChatGPT

    //targetHeading = atan(x-X/y-Y)*180/3.14 - 90;

    printf("TH: %f | ",targetHeading);


 /*
    if (x-X >=0 && y-Y >= 0){ // **** Change "and" to "or"
      error = sqrt((X-x)*(X-x) + (Y-y)*(Y-y));
    }
    else{
      error = -sqrt((X-x)*(X-x) + (Y-y)*(Y-y));
    }
    */
    error = sqrt((X-x)*(X-x) + (Y-y)*(Y-y));

    kturn = kturn0 * error/(error0);


    //error = (x-X)*cos((-targetHeading)*3.14/180) + (y-Y)*sin((-targetHeading)*3.14/180);
    
    total_error += 0.001 * error;
    total_error_h +=  0.001 * terror;

    if(fabs(total_error) > 200){
      total_error = sgn(total_error) * 200;
    }

    if((fabs(error)) < 3.5 || fabs(y-Y) < 3 || fabs(x-X) < 3){// 3 cm of error allowed in this movement
      arrived = 1;
      break;
    }

    terror = targetHeading - H;
    if(fabs(terror)<4){
      total_error_h = 0;
    }


    //turnPower = ktp * terror + kti * total_error_h + ktd * (terror - last_terror);
    turnPower = kturn * terror;
    output = kp * error + ki * total_error - kd * fabs(error - last_error);

    if(output > 100 ){
      output = 100;
    }
    else if(output < -100){
      output = -100;
    }

    if(turnPower > 100 ){
      turnPower = 100;
    }
    else if(turnPower < -100){
      turnPower = -100;
    }

    last_error = error;
    last_terror = terror;

    /*
    if(turnPower >= 0){
      Move((output) * pow / 100 , ( output - 1.13 * turnPower) * pow / 100);//0.3 0.7
    }
    else if(turnPower < 0){
      Move((output - 1.13 * turnPower) * pow / 100 , (output) * pow / 100);//0.3 0.7
    }
    */
    Move((0.7 * output + 1.05 * turnPower) * pow / 100 , (0.7 * output - 1.05 * turnPower) * pow / 100);


    task::sleep(10);
  }
  //printf("\n    time = %f      ",t);
  Move(0, 0);
}
