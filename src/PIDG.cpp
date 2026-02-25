#include "vex.h"
#include "Move.h"
#include "PID.h"
#include <ctime>
#include "LPF.h"
#include "robot-config.h"


using namespace vex;
using signature = vision::signature;


int sgn(double d)//Return the sign of a given value
{
  if (d<0) return -1;
  else if (d==0) return 0;
  else return 1;
}

void turn(int turnpwm,int forwardpwm)
//While the forwarding part is not used anywhere
{
  float turnpower = turnpwm*12/127; 
  float forwardpower = forwardpwm*12/127;
  Left_Motor1.spin(fwd, forwardpower + turnpower, volt);
  Left_Motor2.spin(fwd, forwardpower + turnpower, volt);
  Left_Motor3.spin(fwd, forwardpower + turnpower, volt);

  Right_Motor1.spin(fwd, forwardpower - turnpower, volt);
  Right_Motor2.spin(fwd, forwardpower - turnpower, volt);
  Right_Motor3.spin(fwd, forwardpower - turnpower, volt);
}

void forward_(double leftpower,double rightpower)
{
  if(leftpower > 100){
    leftpower = 100;
  }
  else if (leftpower < -100) {
    leftpower = -100;
  }
  if(rightpower > 100){
    rightpower = 100;
  }
  else if (rightpower < -100) {
    rightpower = -100;
  }
  
  double left = leftpower *12/127;
  double right = rightpower*12/127;
  Left_Motor1.spin(fwd, left, volt);
  Left_Motor2.spin(fwd, left, volt);
  Left_Motor3.spin(fwd, left, volt);

  Right_Motor1.spin(fwd, right, volt);
  Right_Motor2.spin(fwd, right, volt);
  Right_Motor3.spin(fwd, right, volt);
}

void EncoderReset(void)
{
  Left_Motor1.resetPosition();
  Left_Motor2.resetPosition();
  Left_Motor3.resetPosition();

  Right_Motor1.resetPosition();
  Right_Motor2.resetPosition();
  Right_Motor3.resetPosition();

  Left_Motor1.setPosition(0,deg);
  Right_Motor1.setPosition(0,deg);
  Left_Motor2.setPosition(0,deg);
  Right_Motor2.setPosition(0,deg);
  Left_Motor3.setPosition(0,deg);
  Right_Motor3.setPosition(0,deg);

  drive_enc = 0;
}

void Turn(float target, int timeout){
  is_executing_movements = 1;
  target_H = target;

  float kp;
  float ki;
  float kd;
  float maxtime;

  if(fabs(target - H) <= 45){
    kp = 1.5; //1.3
    ki = 0.05; //0.05
    kd = 3.5; //3.5
    maxtime = 850;
  }
  if(fabs(target - H) <= 75 ){
    kp = 1.2; //0.95
    ki = 0.03; //0.03
    kd = 2.8; //2.8
    maxtime = 850;
  }
  else if(fabs(target - H) <= 95){
    kp = 0.95; //0.9 
    ki = 0.02; // 0.013
    kd = 2.2; // 2.2  
    maxtime = 850; 
  }
  else if(fabs(target - H) <= 125){
    kp = 0.8; // 0.61
    ki = 0.03; // 0.02
    kd = 0.9; // 0.97
    maxtime = 900;
  }
  else if(fabs(target - H) <= 145){
    kp = 0.74; // 0.61
    ki = 0.029; // 0.02
    kd = 0.92; // 0.97
    maxtime = 900;
  }
  else if(fabs(target - H) <= 175){
    kp = 0.75; // 0.5
    ki = 0.02; // 0.02
    kd = 2; // 0.8
    maxtime = 1200;
  }
  else{
    kp = 1;
    ki = 0.03;
    kd = 2;
    maxtime = 2000;
  }
maxtime = timeout;
  double total_error = 0;
  int timeused = 0;
  float errortolerance = 1;
  float lim =127;
  float error;
  float lasterror = 0;
  float v = 0;
  float pow;


  float evaluation = 0;
  float overshoot = 0;
  float settlingTime = 0;
  float steadyStateError = 0;

  float sgn_initial_error = sgn(target - H);
  float first_pass = 0;

  while (1)
  {

    timeused += 20;

    error = target - H;
    v = (error - lasterror);
    lasterror = error;

    total_error += error * 0.1;
    if(fabs(total_error) >= 200 || error * sgn(error) < 5){
      total_error = 0;
    }

    pow = kp * error + kd * v + ki * total_error;
    pow = fabs(pow) > lim ? sgn(pow) * lim : pow;

    if (fabs(error) < 5 && (fabs(Inertial_1.gyroRate(zaxis,dps)) + fabs(Inertial_1.gyroRate(zaxis,dps))) / 2 < 2)
    {
     break;
    }
    if(timeused > maxtime){
     break;
    }
    
    turn(pow,0);
    task::sleep(20);
  }
  turn(0, 0);
  Move_Stop_hold();
  is_executing_movements = 0;
}

void Turn_Beta(float target){
  float kp = 1.5;
  float ki = 0;
  float kd = 15;
  double power = 100;
  double endV = 20;
  double total_error = 0;
  int timeused = 0;
  float dtol = endV;
  float errortolerance = 1;
  float lim =127;
  float error = target - H;
  float lasterror;
  float v = 0;
  bool arrived = 0;
  float pow;
  lasterror = error;
  arrived = error == 0;

  while (!arrived)
  {
    timeused += 10;

    error = target - H;

    v = (error - lasterror);
    lasterror = error;
    total_error += error * 0.01;
    if(fabs(total_error) >= 200){
      total_error = sgn(total_error) * 200;
    }
    if(fabs(error) < 1){
      total_error = 0;
    }


    pow = kp * error + kd * v + ki * total_error;
    pow = fabs(pow) > lim ? sgn(pow) * lim : pow;
    pow *= power/100;// Adjust turning speed

    if (fabs(Inertial_1.gyroRate(zaxis,dps)) < 0.2 && timeused > 200)
    {
      break;
    }
    if(timeused > 1000){
      break;
    }
    
    turn(pow,0);
    task::sleep(10);
  }
  turn(0, 0);
  Move_Stop_hold();
}

/*
void pidMove(float maxPower, float target){ 
  //target /= 3.25 * 2.54;
  //target *= 360; 

  // 获取初始位置：左右电机的平均角度
float currentRight_0 = (Right_Motor1.position(deg) + Right_Motor2.position(deg) + Right_Motor3.position(deg)) / 3.0;
float currentLeft_0  = (Left_Motor1.position(deg) + Left_Motor2.position(deg) + Left_Motor3.position(deg)) / 3.0;
float currentRight;
float currentLeft;
float fullTime = 3000;
// 计算初始误差
float errorRight = target - 0;
float errorLeft  = target - 0;
float lastErrorRight = errorRight;
float lastErrorLeft  = errorLeft;
float accumulatedErrorRight = 0.0;
float accumulatedErrorLeft  = 0.0;

float turnpower = 0;
float o = H;
float t = 0;

while (t < fullTime) {
// 更新左右侧电机的当前位置
currentRight = (Right_Motor1.position(deg) + Right_Motor2.position(deg)+ Right_Motor3.position(deg)) / 3.0 - currentRight_0;
currentLeft  = (Left_Motor1.position(deg) + Left_Motor2.position(deg) + Left_Motor3.position(deg)) / 3.0 - currentLeft_0;

// 重新计算误差
errorRight = target - currentRight;
errorLeft  = target - currentLeft;

// 累计积分项：防止误差过大时积分失控，这里只在误差较小时累加
if (fabs(errorLeft) < 200) {
  accumulatedErrorLeft += errorLeft;
}
if (fabs(errorRight) < 200) {
  accumulatedErrorRight += errorRight;
}

// 计算左右侧的 PID 输出
float outputLeft  = 0.35 * errorLeft + 0 * accumulatedErrorLeft + 25 * (errorLeft - lastErrorLeft);
float outputRight = 0.35 * errorRight + 0 * accumulatedErrorRight + 25 * (errorRight - lastErrorRight);
// 将左右输出取平均作为最终输出
float output = (outputLeft + outputRight) / 2.0;

// 限制输出功率：不允许超过最大功率 maxPower
if (fabs(output) > maxPower) {
  output = (target > 0) ? maxPower : -maxPower;
}

// 起步阶段（前150毫秒）限制输出，避免突变
if (t < 150 && fabs(maxPower) > 60) {
  output = (target > 0) ? 60 : -60;
}

// 如果设定功率较大，确保输出有最低限度（防止因输出过小而控制不力）
if (fabs(maxPower) > 30) {
  if (output > 0 && output < 30) {
      output = 30;
  } else if (output < 0 && output > -30) {
      output = -30;
  }
}

turnpower = 2 * (o - H);

// 控制左右电机运动：输出相同功率，保持直行
Move(output + turnpower, output - turnpower);

// 更新上一周期误差，用于微分计算
lastErrorLeft  = errorLeft;
lastErrorRight = errorRight;

// 如果误差足够小，认为已经到达目标，提前退出
if (fabs(errorLeft) < 10 || fabs(errorRight) < 10) {
  break;
}
task::sleep(10);
t += 10;
}

// 停止电机，保持当前位置
Move_Stop_hold();
}
*/

void PIDGMove(double enc , double o, double timeout){ 
  is_executing_movements = 1;
  int time=0;
  double menc = 0;
  double turnpower;
  double pow;
  double last_error = 0;
  double total_error = 0;
  double error = enc - menc;
  
  double kp = 0.1;
  double ki = 0.01 ;
  double kd = 1.1;

  double e0;
  e0 = drive_enc;

  while(true)
  {
    time += 20;

    menc = drive_enc - e0; // the unit of menc is in degrees, which means 1 degree corresponds to 1/360*3.25(wheel diameter)*pi*2.5(convert inches to cm)
    //menc = Low_Pass_Filter_enc(menc, 0.3);

    // Integration
    total_error += enc - menc;
    if(fabs(total_error) >= 150){//set the limit for integral
      total_error = 0;
    }

    // PID output
    pow = kp * fabs(enc - menc) + ki * total_error - kd * (error - last_error); // -last_pow is D
    last_error = error;

    turnpower = 0.04*(o - H)*pow;

    if (fabs(enc-menc)<3 )
    {
      total_error = 0;
    }

    // An alternative breaking method, using the power produced by PID
    if(sgn(pow)*pow < 1 || time > timeout || fabs(menc) >= enc){
        break;
    }

    if (enc < 0)
    {
      forward_(-pow+turnpower,-pow-turnpower);
    }
    else
    {
      forward_(pow+turnpower,pow-turnpower);
    }
    task::sleep(20);
  }
  forward_(0,0);//Reset motor voltage
  is_executing_movements = 0;
}

void GyroMove( double power, double enc , double o){ 
  //Update global variables cooridnates and heading
//  Basic trignometry makes sure that een when robot is not moving 
//  on grids, expected x, y coordinates are sitll updated 
  is_executing_movements = 1;
  int time=0;
  float menc;
  float turnpower;
  double pow = power;
  
  double e0 = drive_enc;

  target_H = o;
  
  while(true)
  {
    time += 20;
    menc = drive_enc - e0;
    //menc = Low_Pass_Filter_enc(menc, 0.7); 

    turnpower = 0.85 * (o - H);


    if (fabs(enc)-fabs(menc) < 10 || time > 6000)
    {
      break;
    }

    if (enc > 0)
    {
      forward_(pow+turnpower,pow-turnpower);
    }
    else
    {
      forward_(-pow+turnpower,-pow-turnpower);
    }
    task::sleep(20);
  }
  forward_(0,0);
  printf("menc: %f\n",menc);
  is_executing_movements = 0;
}

void GMove( double power, double enc){ 
  //Update global variables cooridnates and heading
//  Basic trignometry makes sure that een when robot is not moving 
//  on grids, expected x, y coordinates are sitll updated 
  is_executing_movements = 1;
  int time=0;
  float menc;
  float turnpower;
  double pow = power;
  
  double e0 = drive_enc;
  
  while(true)
  {
    time += 20;
    menc = drive_enc - e0;
    //menc = Low_Pass_Filter_enc(menc, 0.7); 

    turnpower = 0;


    if(fabs(enc)-fabs(menc) < 10)
    {
      break;
    }
    if(time > 2000){
      break;
    }

    if (enc > 0)
    {
      forward_(pow+turnpower,pow-turnpower);
    }
    else
    {
      forward_(-pow+turnpower,-pow-turnpower);
    }
    task::sleep(20);
  }
  forward_(0,0);
  is_executing_movements = 0;
}

void Move_to_yellow(double enc, double pow){
  double menc0 = drive_enc;
  double menc1 = drive_enc - menc0;
  is_executing_movements = 1;
  double turnPow = 0;
  double kt;

  double t = 0;

  while(fabs(menc1) <= fabs(enc)){
    Vision_back.takeSnapshot(Vision__SIG_1);

    menc1 = drive_enc - menc0;

    if(Vision_back.objectCount > 0){
      turnPow = Vision_back.largestObject.centerX - 158;
        kt = 1;
      turnPow  = turnPow * kt;
    }

    if(enc > 0){
      Move((100 - turnPow)*pow/100, (100 + turnPow)*pow/100);
    }
    else{
      Move((-100 - turnPow)*pow/100, (-100 + turnPow)*pow/100);
    }

    if(fabs(menc1 - enc) <= 5 || (Vision_back.largestObject.width > 170)){
      break;
    }

    task::sleep(30);
  }

  Move(0,0);
  is_executing_movements = 0;
}

void Move_to_yellow_front(double enc, double pow){
  double menc0 = drive_enc;
  double menc1 = drive_enc - menc0;
  is_executing_movements = 1;
  double turnPow = 0;
  double kt;

  double t = 0;

  while(fabs(menc1) <= fabs(enc)){
    Vision_front.takeSnapshot(Vision__SIG_1);

    menc1 = drive_enc - menc0;

    if(Vision_front.objectCount > 0){
      turnPow = Vision_front.largestObject.centerX - 158;
        kt = 1;
      turnPow  = turnPow * kt;
    }

    if(enc > 0){
      Move((100 - turnPow)*pow/100, (100 + turnPow)*pow/100);
    }
    else{
      Move((-100 - turnPow)*pow/100, (-100 + turnPow)*pow/100);
    }

    if(fabs(menc1 - enc) <= 5 || (Vision_front.largestObject.width > 170)){
      break;
    }

    task::sleep(30);
  }

  Move(0,0);
  is_executing_movements = 0;
}

void Move_to_blue(double enc, double pow){
  double menc0 = drive_enc;
  double menc1 = drive_enc;
  is_executing_movements = 1;
  double turnPow = 0;
  double kt;

  double t = 0;
  menc1 = drive_enc - menc0;

  while(fabs(menc1) <= fabs(enc)){
    Vision_front.takeSnapshot(Vision__SIG_3);

    menc1 = drive_enc - menc0;

    if(Vision_front.objectCount > 0){
      turnPow = Vision_front.largestObject.centerX - 158;
        kt = 1;
      turnPow  = turnPow * kt;
    }

    if(enc > 0){
      Move((100 + turnPow)*pow/100, (100 - turnPow)*pow/100);
    }
    else{
      Move((-100 + turnPow)*pow/100, (-100 - turnPow)*pow/100);
    }

    if(fabs(menc1 - enc) <= 5){
      break;
    }

    task::sleep(30);
    printf("object count: %d\n", Vision_front.objectCount);
  }

  Move(0,0);
  is_executing_movements = 0;
}

void Move_to_red(double enc, double pow){
  double menc0 = drive_enc;
  double menc1 = drive_enc;

  double turnPow = 0;
  double kt;
  is_executing_movements = 1;
  double t = 0;
  menc1 = drive_enc - menc0;

  while(fabs(menc1) <= fabs(enc)){
    Vision_front.takeSnapshot(Vision__SIG_3);

    menc1 = drive_enc - menc0;

    if(Vision_front.objectCount > 0){
      turnPow = Vision_front.largestObject.centerX - 158;
        kt = 1;
      turnPow  = turnPow * kt;
    }

    if(enc > 0){
      Move((100 + turnPow)*pow/100, (100 - turnPow)*pow/100);
    }
    else{
      Move((-100 + turnPow)*pow/100, (-100 - turnPow)*pow/100);
    }

    if(fabs(menc1 - enc) <= 5 || (Vision_front.largestObject.width > 170)){
      break;
    }

    task::sleep(30);
  }

  Move(0,0);
  is_executing_movements = 0;
}

void follow_left_wall(double enc, double distance){
  is_executing_movements = 1;

  float kturn = 0;

  int time=0; 
  double menc;
  double turnpower;
  double pow;
  double last_pow = 0;
  double total_error = 0;
  
  double kp = 0.001;//0.0009
  double ki = 0.0001 ;//0.000001 
  double kd = 0.0025; // 0.002 If necessary, make kd actually have a value so that the settling time can be shorter

  if(enc < 0){
    kp = 0.001;
    ki = 0.0001 ;
    kd = 0.0025;
  }

  double e0;
  e0 = drive_enc;

  if(fabs(distance - L_dis) <= 15){
      kturn = 0.022;
    }
    else if(fabs(distance - L_dis) <= 45){
      kturn = 0.022;
    }

  while(true)
  {
    time += 20;

    menc = drive_enc - e0;
    //menc = Low_Pass_Filter_enc(menc, 0.3);

    // Integration
    total_error += enc - menc;
    if(fabs(total_error) >= 1000){//set the limit for integral
      total_error = 1000 * sgn(total_error);
    }

    // PID output
    pow = 100 * (kp * fabs(enc - menc) + ki * total_error - kd * fabs(last_pow)); // -last_pow is D
    last_pow = pow;

    turnpower = kturn * (distance - L_dis) * pow;
    if(fabs(pow) > 100){
      turnpower = kturn * (distance - L_dis) * 100;
    }

    if (fabs(enc-menc)<3 )
    {
      total_error = 0;
    }

    // An alternative breaking method, using the power produced by PID
    if(fabs(menc) > fabs(enc)){
      break;
    }


    if (enc < 0)
    {
      forward_(-pow+turnpower,-pow-turnpower);
    }
    else
    {
      forward_(pow+turnpower,pow-turnpower);
    }
    task::sleep(20);
    //printf("menc: %f\n", menc);
  }
  forward_(0,0);//Reset motor voltage
  is_executing_movements = 0;
}

void follow_right_wall(double enc, double distance){
  is_executing_movements = 1;

  float kturn = 0;

  int time=0; 
  double menc;
  double turnpower;
  double pow;
  double last_pow = 0;
  double total_error = 0;
  
  double kp = 0.001;//0.0009
  double ki = 0.0001 ;//0.000001 
  double kd = 0.0025; // 0.002 If necessary, make kd actually have a value so that the settling time can be shorter

  if(enc < 0){
    kp = 0.001;
    ki = 0.0001 ;
    kd = 0.0025;
  }

  double e0;
  e0 = drive_enc;

  if(fabs(distance - R_dis) <= 15){
      kturn = 0.022;
    }
    else if(fabs(distance - R_dis) <= 45){
      kturn = 0.022;
    }

  while(true)
  {
    time += 20;

    menc = drive_enc - e0;
    //menc = Low_Pass_Filter_enc(menc, 0.3);

    // Integration
    total_error += enc - menc;
    if(fabs(total_error) >= 1000){//set the limit for integral
      total_error = 1000 * sgn(total_error);
    }

    // PID output
    pow = 100 * (kp * fabs(enc - menc) + ki * total_error - kd * fabs(last_pow)); // -last_pow is D
    last_pow = pow;

    turnpower = kturn * (distance - R_dis) * pow;
    if(fabs(pow) > 100){
      turnpower = kturn * (distance - R_dis) * 100;
    }

    if (fabs(enc-menc)<3 )
    {
      total_error = 0;
    }

    // An alternative breaking method, using the power produced by PID
    if(fabs(menc) > fabs(enc)){
      break;
    }


    if (enc < 0)
    {
      forward_(-pow+turnpower,-pow-turnpower);
    }
    else
    {
      forward_(pow+turnpower,pow-turnpower);
    }
    task::sleep(20);
    //printf("menc: %f\n", menc);
  }
  forward_(0,0);//Reset motor voltage
  is_executing_movements = 0;
}

void forward_until_dis(double distance, double pow){
  is_executing_movements = 1;

  while(L_dis > distance){
    forward_(pow,pow);
    task::sleep(20);
  }
  forward_(0,0);
  is_executing_movements = 0;
}

void pid_until_dis(float target_dis, bool isfront, double o, int timeout){
  is_executing_movements = 1;
  int time=0;
  double turnpower;
  double pow;
  double last_error = 0;
  double total_error = 0;
  double error = 0;

  if(isfront){
    error = -(target_dis - front_dis);
    front_dis = dis_front.objectDistance(mm);
  }
  else{
    error = target_dis - back_dis;
    back_dis = dis_back.objectDistance(mm);
  }
  
  double kp = 0.17;
  double ki = 0.11;
  double kd = 0.9;


  while(true)
  {
    if(isfront){
    front_dis = dis_front.objectDistance(mm);
  }
  else{
    back_dis = dis_back.objectDistance(mm);
  }
    time += 10;
    if(isfront){
      error = -(target_dis - front_dis);
    }
    else{
      error = target_dis - back_dis;
    }

    // Integration
    total_error += error;
    if(fabs(total_error) >= 100){
      total_error = 100;
    }

    // PID output
    pow = kp * fabs(error) + ki * sgn(error) * fabs(total_error) - kd * (error - last_error) + 2; // -last_pow is D
    last_error = error;

    turnpower = 0.05*(o - H)*pow;

    if (fabs(error)<100 )
    {
      total_error = 0;
    }

    if(fabs(error) < 15 || time > timeout){
        break;
    }

    if(error < 0){
      forward_(-pow+turnpower,-pow-turnpower);
    }
    else{
      forward_(pow+turnpower,pow-turnpower);
    }
    task::sleep(10);
  }
  forward_(0,0);//Reset motor voltage
  is_executing_movements = 0;
}

float slew(float cur, float last, float maxChange) {
  float change = cur - last;
  if (change > maxChange) change = maxChange;
  else if (change < -maxChange) change = -maxChange;
  return last + change;
}

void MoveTo(float tar_x,float tar_y,int fulltime,int min_power){

  const float MaxMovement = 1.3 * sqrt((tar_x - X) * (tar_x - X) + (tar_y - Y) * (tar_y - Y));
  is_executing_movements = 1;
  const float kp_1 = 2;
  const float ki_1 = 0;
  const float kd_1 = 10;
  const float kp_2 = 0.6;
  const float ki_2 = 0;
  const float kd_2 = 10;

  float err_x;//横坐标误差
  float err_y;//纵坐标误差
  float e0 = drive_enc;
  float enc = drive_enc - e0;;

  float dis;//两点间距
  float last_dis;
  float acc_dis = 0;

  float err_ang;//角度差
  float last_ang;
  float acc_ang;

  float power;//直行功率
  float turn_power;//转弯功率
  float last_turn_power=0;
  float last_power=0;
  float power_maxchange=15;//每次循环允许的直行功率最大变化
  float turn_power_maxchange=30;//每次循环允许的直行功率最大变化

  float max_power = 100;
  timer time;

  float cur_x = X;
  float cur_y = Y;

  err_x = tar_x - cur_x;
  err_y = tar_y - cur_y;
  err_ang = atan2f(err_x,err_y) * (180 / 3.1415) - H;
  float last_err = err_ang;
  while(err_ang > 90){
    err_ang -= 180;
  }
  while(err_ang < -90){
    err_ang += 180;
  }
  
  //If the difference in angle is too large
  while(fabs(err_ang) > 20 && MaxMovement > 5){
    cur_x = X;
    cur_y = Y;
    err_x = tar_x - cur_x;
    err_y = tar_y - cur_y;
    err_ang = atan2f(err_x,err_y) * (180 / 3.1415) - H;
    while(err_ang > 90){
      err_ang -= 180;
    }
    while(err_ang < -90){
      err_ang += 180;
    }
    float out = 1.2 * err_ang + 8 * (err_ang - last_err);
    //float out = PID(1.2,0,10).compute(err_ang,last_err,0);
    last_err = err_ang;
    
    Move(out,-out);
  }
  
  timer break_time;
  while(break_time < 40 && time < fulltime && MaxMovement > 5){
    enc = drive_enc - e0;
    if(enc > MaxMovement){
      break;
    }
    cur_x = X;
    cur_y = Y;
    err_x = tar_x-cur_x;
    err_y = tar_y-cur_y;
    float distance = sqrt(err_x*err_x + err_y*err_y);
    dis = distance*cos(atan2f(err_x,err_y) - ((3.1416/180)*H));

    bool close = distance < 10;

    err_ang = atan2f(err_x,err_y) * (180 / 3.1415) - H;
    while(err_ang > 90){
      err_ang -= 180;
    }
    while(err_ang < -90){
      err_ang += 180;
    }

    //直行功率计算
    //PID MOVETO_POWER(2,0,10);
    power = kp_1 * dis + ki_1 * last_dis + kd_1 * (dis - last_dis);
    if(distance < 25){
      max_power = 70;
    }
    if(close){
      max_power = 20;
    }
    //power = std::min(power,max_power);
    if(power > max_power){
      power = max_power;
    }
    else{
      power = power;
    }
    //power = std::max(power,-max_power);
    if(power < min_power){
      power = min_power;
    }
    else{
      power = power;
    }
    if(power < min_power && power > -min_power){
      power = -10;
    }else if(power > min_power && power < min_power){
      power = 10;
    }
    power = slew(power,last_power,power_maxchange);

     //旋转功率计算
    //PID MOVETO_TURN_POWER(0.6,0,10);
    turn_power = kp_2 * err_ang + ki_2 * last_ang + kd_2 * (err_ang - last_ang);
    if(turn_power > 100){
      turn_power = 100;
    }else if(turn_power<-100){
      turn_power=-100;
    }
    if(fabs(err_ang) > 3 && !close){
      if(turn_power > 0 && turn_power < 40){
        turn_power = 40;
      }else if(turn_power < 0 && turn_power > -40){
        turn_power = -40;
      }
    }
    if(fabs(err_ang) < 4 || close){
      turn_power = 0;
    }
    //if(turn_power != 0)turn_power=slew(turn_power,last_turn_power,turn_power_maxchange);
    //旋转功率计算
    double lpower = power+turn_power;
    double rpower = power-turn_power;
    double maxpower = (fabs(lpower)>fabs(rpower))?lpower:rpower;
    double ratio = fabs(maxpower/100);
    if(ratio > 1){
      lpower/=ratio;
      rpower/=ratio;
    }
    Move(lpower,rpower);

    if(distance > 4){
      break_time = 0;
    }

    last_dis = dis;
    acc_dis += dis;
    last_ang = err_ang;
    acc_ang += err_ang;
    last_power = power;
    last_turn_power = turn_power;
    
    Brain.Screen.printAt(100,60,"err_dis: %f",sqrt(err_x*err_x + err_y*err_y));
    Brain.Screen.printAt(100,80,"err_ang: %f",err_ang);
    Brain.Screen.printAt(100,100,"power: %f",power);
    Brain.Screen.printAt(100,120,"turnpower: %f",turn_power);
    Brain.Screen.printAt(100,140,"x: %f",X);
    Brain.Screen.printAt(100,160,"y: %f",Y);

    task::sleep(5);
  }
  Move_Stop_hold();
  target_H = H;
  is_executing_movements = 0;
}





