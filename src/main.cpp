#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Left_Motor1          motor         9               
// Left_Motor2          motor         7               
// Left_Motor3          motor         10              
// Right_Motor1         motor         2               
// Right_Motor2         motor         6               
// Right_Motor3         motor         5               
// Inertial             inertial      8               
// Catch                digital_out   H               
// Up                   digital_out   A               
// Roller_Up            motor         3               
// Roller_Intake        motor         1               
// Arm                  motor         11              
// Whiteflag            digital_out   G               
// Opt                  optical       14              
// RingSelection        digital_out   D               
// Inertial20           inertial      20              
// Inertial2            inertial      19              
// Horizontal           rotation      15              
// Vertical             rotation      4               
// ArmPos               rotation      12              
// Up2                  digital_out   B               
// Vision               vision        13              
// Left                 rotation      18              
// Right                rotation      17              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "PID.h"
#include "Move.h"
#include "Auxiliary.h"
#include "LPF.h"
#include "Auton.h"
#include "iostream"
#include "cmath"

using namespace vex;

competition Competition;

float lm1_enc = 0; // Setting up variables for motor encoders
float rm1_enc = 0;
float lm2_enc = 0;
float rm2_enc = 0;
float lm3_enc = 0;
float rm3_enc = 0;

double X;
double Y;
double H;
void pre_auton(void) 
{
  vexcodeInit();
  X = 0;
  Y = 0;
  H = 0;
  EncoderReset();
  Inertial_1.calibrate();
  //Inertial_2.calibrate();
  wait(2200, msec);
  Controller1.rumble("..");
  d_left.resetPosition();
  d_right.resetPosition();
}

bool stopThread = 0;
double dt = 10;
double drive_enc;
float L_dis = 0;
float R_dis = 0;
float front_dis = 0;
float back_dis = 0;
double H_1 = 0;
double H_2 = 0;
double h_1 = 0; // Last heading value
double h_2 = 0;
void updates(){ 
  while(!stopThread){

    H = Inertial_1.rotation(deg);
    /*
    // Double Inertial Heading Calculation
    if(Inertial_1.installed() && Inertial_2.installed()){
      H_1 = Inertial_1.rotation(deg);
      H_2 = Inertial_2.rotation(deg);
      if(fabs(H_1 - H_2) >= 10){
        printf("Inertial not synced");
        if(fabs(H_1 - H) < fabs(H_2 - H)){
          H = H_1;
          h_1 = H_1;
        }
        else{
          H = H_2;
          h_2 = H_2;
        }
      }
      else{
        H = (H_1 + H_2) / 2;
        h_1 = H_1;
        h_2 = H_2;
      }
    }
    else if(!Inertial_1.installed()){
      H_2 = Inertial_2.rotation(deg);
      H = H_2;
      h_2 = H_2;
    }
    else{
      H_1 = Inertial_1.rotation(deg);
      H = H_1;
      h_1 = H_1;
    }
    */

    lm2_enc = d_left.position(deg);
    rm2_enc = d_right.position(deg);
    drive_enc = (lm2_enc + rm2_enc) / 2;
    //Enc_pos();


    // Display Debugger
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("drive_enc: %f", drive_enc);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("right: %f  mm", R_dis);
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("front: %f   back: %f", front_dis, back_dis);
    //printf("H: %f  front: %f  back: %f \n", H, front_dis, back_dis);

    task::sleep(dt);
  }
}

double target_H = 0;
bool is_executing_movements = 0;

void auto_correct(){
  while(!stopThread){
    if(Competition.isAutonomous()){
      if(!is_executing_movements && fabs(H-target_H) > 1){
        turn(-21 * sgn(H-target_H),0);
      }
    }
    task::sleep(dt);
  }
}


double thread_select = 1;
double down = 0;
double up = 0;
double middle = 0;
//double opt_val = Opt.hue();
//double dist = dis.objectDistance(mm);
bool selecting = 0;
bool intake_protection = 1;
bool uptake_protection = 1;
/*
void select(){
  Opt.setLightPower(100,pct);
  //Opt.integrationTime(50);
  double lagst_opt = 0;
  while(1){
    //Opt.setLightPower(100,pct);
    if (lagst_opt < opt_val)
    {
      lagst_opt = opt_val;
    }
    //printf("Opt: %f\n",lagst_opt);
    //dist = dis.objectDistance(mm);
    if(Roller_Down.torque(Nm)> 0.57 && uptake_protection){
      for(int i = 0; i < 7; i++){
        //Roller_Down.spin(reverse,30,pct);
        wait(10,msec);
      }
    }
    else if(thread_select == 1){
      opt_val = Opt.hue();
      if(side == 1 && opt_val > 55 && opt_val < 350){
        selecting = 1;
      }
      else if(side == 0 && ((opt_val > 1 && opt_val < 20) || (opt_val > 330 && opt_val < 360))){
        selecting = 1;
      }
    }

    if(!selecting){
      if(down != 0) Roller_Down.spin(fwd,down * 128,volt);
      else Roller_Down.stop(coast);
      if(up != 0) Roller_Up.spin(fwd,up * 128,volt);
      else Roller_Up.stop(coast);
      if(middle != 0) Roller_Middle.spin(fwd, middle * 128,volt);
      else Roller_Middle.stop(coast);
    }

    if(selecting){
      select_ball.set(1);
      Roller_Down.spin(fwd,30,pct);
      Roller_Middle.spin(fwd,20,pct);
      Roller_Up.stop(coast);
      for(int i = 0; i < 200; i++){
        selecting = 0;
        wait(10,msec);
      }
      Roller_Down.stop(coast);
      select_ball.set(0);
      /*
      for(int i = 0; i < 4; i++){
        selecting = 0;
        Roller_Up.spin(fwd,12.8,volt);
        wait(10,msec);
      }
      for (int i = 0; i < 50; i++)
      {
        selecting = 0;
        Roller_Up.spin(reverse,12.8,volt);
        wait(10,msec);
      }
    }

    // Enabling/disabling ring selection
    if(thread_select == 0 && Controller1.ButtonX.pressing()){
      Controller1.rumble("----");
      waitUntil(!Controller1.ButtonX.pressing());
      thread_select = 1;
    }
    else if (thread_select == 1 && Controller1.ButtonX.pressing()) {
      Controller1.rumble("....");
      waitUntil(!Controller1.ButtonX.pressing());
      thread_select = 0;
    }

      task::sleep(10);
  }
}
*/

/*
int autonToRun = 0;
void display(){
class Button
{
  public:
    int x, y, width, height;
    std::string text;
    vex::color buttonColor, textColor;
    
    Button(int x, int y, int width, int height, std::string text, vex::color buttonColor, vex::color textColor)
    : x(x), y(y), width(width), height(height), text(text), buttonColor(buttonColor), textColor(textColor){}

    void render()
    {
      Brain.Screen.drawRectangle(x, y, width, height, buttonColor);
      Brain.Screen.printAt(x + 10, y + 20, false, text.c_str());
    }

    bool isClicked()
    {
      if(Brain.Screen.pressing() && Brain.Screen.xPosition() >= x && Brain.Screen.xPosition() <= x + width &&
      Brain.Screen.yPosition() >= y && Brain.Screen.yPosition() <= y + width) return true;
      return false;
    }
};

Button autonButtons[] = {
  Button(10, 10, 100, 30, "b_r_16", vex::blue, vex::blue),
  Button(120, 10, 100, 30, "b_rush", vex::blue, vex::blue),
  Button(240, 10, 100, 30, "b_l_5", vex::blue, vex::blue),
  Button(10, 50, 100, 30, "b_6", vex::blue, vex::blue),
  Button(120, 50, 100, 30, "b_l_14", vex::blue, vex::blue),
  Button(10, 100, 100, 30, "r_l_16", vex::red, vex::red),
  Button(120, 100, 100, 30, "r_rush", vex::red, vex::red),
  Button(240, 100, 100, 30, "r_solo", vex::red, vex::red),
  Button(10, 140, 100, 30, "r_6", vex::red, vex::red),
  Button(120, 140, 100, 30, "r_r_14", vex::red, vex::red),

};
int prevAutonToRun = 0; // Variable to store the previous selection
  while(!stopThread){
    Brain.Screen.clearScreen(vex::black);
    if(!Competition.isEnabled())
    {
      for (int i = 0; i < 10; i++) {
        if (autonButtons[i].isClicked()) {
          // Revert previous selection to default color
          if (prevAutonToRun > 0) { // Check if there was a previous selection
            autonButtons[prevAutonToRun - 1].buttonColor =
                (prevAutonToRun - 1 <= 4) ? vex::blue : vex::red;
          }
          // Set clicked button to green
          autonButtons[i].buttonColor = vex::green;
          autonToRun = i + 1;
          prevAutonToRun = autonToRun; // Update previous selection
        }
      }

      // Render all buttons after updating colors
      for (int i = 0; i < 10; i++) {
        // Ensure selected button stays green, others revert to default
        if (autonToRun == i + 1) {
          autonButtons[i].buttonColor = vex::green;
        } else {
          autonButtons[i].buttonColor = (i <= 4) ? vex::blue : vex::red;
        }
        autonButtons[i].render();
      }
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print("Auton: %d", autonToRun);
    }
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("%f", X);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%f", Y);
    Brain.Screen.render();
    task::sleep(20);
    //Controller1.Screen.setCursor(3, 1);
    //Controller1.Screen.print("%f", Inertial.heading());
  }
}
*/

char opcolor = 'b';

char get_color(){
    optical::rgbc rgb = Optical_sensor.getRgb();
    int R = rgb.red;
    int B = rgb.blue;

    // 太暗直接忽略
    if (R + B < 300) return '-';

    float ratio = (float)R / (float)(B + 1);

    if (ratio > 1.35) return 'r';   // 红明显多
    if (ratio < 0.80) return 'b';   // 蓝明显多

    return '-'; // 不确定
}

void Intake(int num,float Power){
  motor* p;
  if(num == 1) p = &intake1;
  else if(num == 2) p = &intake2;
  else if(num == 3) p = &intake3;
  else if(num == 4) p = &intake4;
  if(Power == 0) p->stop(hold);
  else p->spin(fwd, 0.128 * Power, voltageUnits::volt);
}

int powers[4] = {0,0,0,0};
int ballway_mode = 0; //
int count_flag = true;  //数球检测松开
bool sort_flag = false;
timer sort_time;
bool sort_flag_op = true;
timer sort_time_op;


int ballway_progress(){
  while (true)
  {
    if(ballway_mode == 0){
      powers[0] = 0;
      powers[1] = 0;
      powers[2] = 0;
      powers[3] = 0;
     }
    else if(ballway_mode == 1){
      powers[0] = 100;powers[1] = 100;powers[2] = 100;powers[3] = 100;
     }//手动收球
    else if(ballway_mode == 2){
      powers[0] = -100;powers[1] = -100;
      if(Ballway_Distance_Mid.objectDistance(mm) <60)
      {
        powers[2] = -100;powers[3] = -100;
      }
      else
      {
        powers[2] = -100;powers[3] = -100;
      }     
    }
    else if(ballway_mode == 3){
      powers[0] = 100;powers[1] = 100;powers[2] = 100;powers[3] = -100;
    }
    else if(ballway_mode == 4){
       powers[0] = 100;
      if (Ballway_Distance_Mid.objectDistance(mm) < 60 && Ballway_Distance_Up.objectDistance(mm) > 60)
      {
        powers[1]=40;
        powers[2]=38;
        powers[3]=10;
      }
      //else if (Ballway_Distance_Mid.objectDistance(mm) < 60  && Ballway_Distance_Up.objectDistance(mm) > 60  && get_color() == opcolor)
      //{
        //powers[1]=70;
        //powers[2]=-22;
        //powers[3]=0;
      //}
      else if (Ballway_Distance_Mid.objectDistance(mm) < 60 && Ballway_Distance_Up.objectDistance(mm) < 60)
      {
        powers[1]=20;
        powers[2]=15;
        powers[3]=0;
      }
      //else if (Ballway_Distance_Mid.objectDistance(mm) < 60 && Ballway_Distance_Up.objectDistance(mm) < 60 && get_color() == opcolor)
      //{
        //powers[1]=70;
        //powers[2]=-22;
        //powers[3]=0;
      //}
      else{
        //powers[3] = -15;
        powers[2] = -5;
        powers[1] = 40;
      }//手动收球
     }
    //else if(ballway_mode == 5){
      //powers[0] = -50;powers[1] = -50;powers[2] = -50;powers[3] = -80;
    //}
    //else if(ballway_mode == 6){
      //powers[0] = 70;powers[1] = 70;powers[2] = 30;powers[3] = -30;
    //}
    if(ballway_mode != 21 && ballway_mode != 5 && ballway_mode != 6 && ballway_mode != 7 && ballway_mode != 8 && ballway_mode != 9 && ballway_mode != 10 && ballway_mode != 11 && ballway_mode != 17 && ballway_mode != 12 && ballway_mode != 15){
      for(int i = 0;i<4;i++) Intake(i+1,powers[i]);
    }
    else if(ballway_mode == 5){
      intake1.spin(fwd,-100,rpm);
      intake2.spin(fwd,-80,rpm);
      intake3.spin(fwd,-60,rpm);
      intake4.spin(fwd,-130,rpm);
    }
    else if(ballway_mode == 6){
      intake1.spin(fwd,125,rpm);
      intake2.spin(fwd,34,rpm);
      intake3.spin(fwd,70,rpm);
      intake4.spin(fwd,-38,rpm);
    }
    else if(ballway_mode == 7){
      intake1.spin(fwd,135,rpm);
      intake2.spin(fwd,115,rpm);
      intake3.spin(fwd,130,rpm);
      intake4.spin(fwd,-110,rpm);
    }
    else if(ballway_mode == 8){
      intake1.spin(fwd,140,rpm);
      intake2.spin(fwd,90,rpm);
      intake3.spin(fwd,110,rpm);
      intake4.spin(fwd,-70,rpm);
    }
    else if(ballway_mode == 9){
      intake1.spin(fwd,-90,rpm);
      intake2.spin(fwd,-70,rpm);
      intake3.spin(fwd,-50,rpm);
      intake4.spin(fwd,-40,rpm);
    }
    else if(ballway_mode == 10){
      intake1.spin(fwd,200,rpm);
      intake2.spin(fwd,100,rpm);
      intake3.spin(fwd,100,rpm);
      intake4.spin(fwd,100,rpm);
    }
    else if(ballway_mode == 11){
      intake1.spin(fwd,120,rpm);
      intake2.spin(fwd,70,rpm);
      intake3.spin(fwd,100,rpm);
      intake4.spin(fwd,-70,rpm);
    }
    else if(ballway_mode == 12){
      intake1.spin(fwd,-200,rpm);
      intake2.spin(fwd,-200,rpm);
      intake3.spin(fwd,0,rpm);
      intake4.spin(fwd,0,rpm);
    }
    else if(ballway_mode == 15){
      intake1.spin(fwd,-100,rpm);
      intake2.spin(fwd,-80,rpm);
      intake3.spin(fwd,0,rpm);
      intake4.spin(fwd,0,rpm);
    }
    else if(ballway_mode == 17){
      intake1.spin(fwd,0,rpm);
      intake2.spin(fwd,115,rpm);
      intake3.spin(fwd,120,rpm);
      intake4.spin(fwd,-105,rpm);
    }
    else if(ballway_mode == 21){
      intake1.spin(fwd,0,rpm);
      intake2.spin(fwd,0,rpm);
      intake3.spin(fwd,200,rpm);
      intake4.spin(fwd,200,rpm);
    }
    task::sleep(10);
  }
  return 0;
}
/* The original ballway control version with color sort
int ballway_progress(){
  while (true)
  {
    if(ballway_mode == 0){
      powers[0] = 0;powers[1] = 0;powers[2] = 0;powers[3] = 0;//停止
     }
    else if(ballway_mode == 1){
      powers[0] = 100;powers[1] = 100;powers[2] = 100;powers[3] = 100;
     }//手动收球
    else if(ballway_mode == 2){
      powers[0] = -100;powers[1] = -100;
      if(Ballway_Distance_Mid.objectDistance(mm) <60)
      {
        powers[2] = 0;powers[3] = 0;
      }
      else
      {
        powers[2] = -100;powers[3] = -100;
      }     
    }
    else if(ballway_mode == 3){
      powers[0] = 100;powers[1] = 100;powers[2] = 100;powers[3] = -100;
    }
    else if(ballway_mode == 4){
       powers[0] = 100;
      if (Ballway_Distance_Mid.objectDistance(mm) < 60 && Ballway_Distance_Up.objectDistance(mm) > 60 && get_color() != opcolor)
      {
        powers[1]=40;
        powers[2]=38;
        powers[3]=5;
      }
      else if (Ballway_Distance_Mid.objectDistance(mm) < 60  && Ballway_Distance_Up.objectDistance(mm) > 60  && get_color() == opcolor)
      {
        powers[1]=70;
        powers[2]=-22;
        powers[3]=0;
      }
      else if (Ballway_Distance_Mid.objectDistance(mm) < 60 && Ballway_Distance_Up.objectDistance(mm) < 60 && get_color() != opcolor)
      {
        powers[1]=10;
        powers[2]=5;
        powers[3]=0;
      }
      else if (Ballway_Distance_Mid.objectDistance(mm) < 60 && Ballway_Distance_Up.objectDistance(mm) < 60 && get_color() == opcolor)
      {
        powers[1]=70;
        powers[2]=-22;
        powers[3]=0;
      }
      else{
        powers[2] = -5;
        powers[1] = 40;
      }//手动收球
     }
    else if(ballway_mode == 5){
      powers[0] = 100;powers[1] = 100;powers[2] = 80;powers[3] = -80;
    }
    else if(ballway_mode == 6){
      powers[0] = 100;powers[1] = 50;powers[2] = 50;powers[3] = -50;
    }
    for(int i = 0;i<4;i++) Intake(i+1,powers[i]);
    task::sleep(10);
  }
  return 0;
}
*/


void autonomous(void) 
{ 
  
  thread autonomous_heading_correction = thread(auto_correct);
  //thread ball_selection = thread(select);
  stopThread = 0;

  //==================== Autonomous Movements Below ==============================
  //Move_to_yellow(-400,60);
  //Move_Stop_brake();
  //solo_AWP();

  /* Test Route - Solo AWP (Not finished yet)
  ballway_mode = 4;
  double_park.set(1);
  PIDGMove(700,-14,3000);
  double_park.set(0);
  Load.set(1);
  GyroMove(40,300,-17);
  Move_Stop_brake();
  Turn(-135);
  Move_to_yellow(-300,40);
  Move(-30,-30);
  task::sleep(400);
  Move_Stop_brake();
  
  ballway_mode = 3;
  task::sleep(300);
  ballway_mode = 4;
  Load.set(0);

  PIDGMove(2100,-140,3000);
  Turn(-180);
  Load.set(1);
  task::sleep(200);
  GyroMove(60,350,-180);
  Move(10,10);
  task::sleep(1500);
  Move_Stop_brake();

  Move_to_yellow(-500,60);
  Move(-30,-30);
  task::sleep(400);
  Move_Stop_brake();

  ballway_mode = 1;
  task::sleep(1000);

  ballway_mode = 4;
  Load.set(0);
  GyroMove(50,1000,-270);

  pid_until_dis(1200,0,-270);
  Move_Stop_brake();

  PIDGMove(1700,-290,3000);
  GyroMove(45,300,-300);
  Move_Stop_brake();

  Turn(-210);
  PIDGMove(1000,-210,3000);
  Turn(-179);
  Load.set(1);
  task::sleep(100);

  GyroMove(40,200,-180);
  Move(20,20);
  task::sleep(500);

  Move_to_yellow(-500,60);
  Move(-30,-30);
  task::sleep(400);
  Move_Stop_brake();

  ballway_mode = 1;
  */

  ballway_mode = 4;
  Move(15,15);
  task::sleep(300);
  Move_Stop_coast();
  for(int i = 0; i < 2; i++){
    Move(-25,-25);
    task::sleep(400);
    Move(37,37);
    task::sleep(600);
  }
  Move(-25,-25);
  task::sleep(400);
  Move(39,39);
  task::sleep(650);

  GyroMove(50,-250,0);
  GyroMove(60,-200,0);
  GyroMove(50,-300,0);
  GyroMove(40,-200,0);
  Move_Stop_hold();
  pid_until_dis(1080,1,0,950);
  Move_Stop_hold();
  //ballway_mode = 0;

   Turn(102,600);
   //ballway_mode = 4;
   PIDGMove(920,108,2000);
   Move_Stop_hold();
   Turn(47,700);
   GyroMove(50,-100,50);

   Move_to_yellow(-100,65);
   Move(-30,-30);
   task::sleep(120);
   Move_Stop_brake();
   Move(-3,-3);

  Load.set(1); // 7 balls in the mid goal
  int t = 0;
  while(Ballway_Distance_Up.objectDistance(mm) < 60 && t <= 700){
    ballway_mode = 5;
    task::sleep(10);
    t += 10;
  }

  ballway_mode = 17;
  task::sleep(160);
  ballway_mode = 7;
  task::sleep(30);
  ballway_mode = 8;
  task::sleep(260);
  Move(-1,-1);
  ballway_mode = 11;
  task::sleep(850);
  ballway_mode = 6;
  Move_Stop_brake();
  task::sleep(1400);
  Move(1,1);
  task::sleep(600);
  Move_Stop_hold();

  ballway_mode = 4;
  GyroMove(80,280,42);

  PIDGMove(1800,55,2000);
  Load.set(1);
  Move_Stop_hold(); 
  Turn(0,400);

  Move_to_blue(200,30); // Clear first loader
  Move_to_blue(200,60);
  Move(30,30);
  task::sleep(400);
  Move(20,20);
  task::sleep(1200);
  Move(25,25);
  task::sleep(400);
  Move_Stop_brake();

  GyroMove(60,-100,-45);
  GyroMove(70,-250,-65);
  while(back_dis > 650){
    Move(-65,-65);
    task::sleep(10);
  }
  Move_Stop_brake();
  GyroMove(80,-700,-2);
  GyroMove(100,-2650,0);
  Load.set(0);
  //GyroMove(72,-280,72);
  GyroMove(52,-700,90);
  GyroMove(30,-330,90);
  Move_Stop_brake();

  Turn(167,450);
  Move_to_yellow(-300,60);
  Move_to_yellow(-100,30);
  Move(-60,-60);
  task::sleep(100);
  ballway_mode = 21;
  task::sleep(100);
  Move_Stop_coast();
  Load.set(1);
  task::sleep(50);
  Move(-5,-5);
  ballway_mode = 1;
  task::sleep(1300);
  ballway_mode = 4;

  GyroMove(100,50,180); // Clear second loader
  Move_to_red(400,40);
  Move(45,45);
  task::sleep(350);
  Move(30,30);
  task::sleep(500);
  Move(15,15);
  task::sleep(200);
  Move(30,30);
  task::sleep(200);
  Move(15,15);
  task::sleep(1200);
  Move_Stop_brake();
  Move_to_yellow(-550,80);
  Move_to_yellow(-390,30);
  Move(-20,-20);
  task::sleep(300);
  Move_Stop_brake();
  ballway_mode = 21;
  task::sleep(50);
  ballway_mode = 1;
  task::sleep(1300);
  // Finiashed fist long goal


  Load.set(0);
  GyroMove(40,300,240);
  GyroMove(100,500,270);
  GyroMove(30,50,270);
  Move_Stop_brake();

  pid_until_dis(1525,0,270,2000);
  Move_Stop_hold();

  Turn(180,650);

  //printf("New test: \n\n");
  ballway_mode = 4; // Clear second parking zone
  GyroMove(55,150,180);
  GyroMove(85,100,180);
  GyroMove(40,150,180);
  Move(30,30);
  task::sleep(130);
  Move(16,16);
  task::sleep(130);
  Move_Stop_brake();
  for(int i = 0; i < 3; i++){
    Move(-26,-26);
    task::sleep(400);
    Move(36,36);
    task::sleep(640);
  }
  GyroMove(45,-200,180);
  GyroMove(70,-350,180);
  GyroMove(40,-150,180);
  GyroMove(30,-100,180);
  ballway_mode = 0;
  //Move_Stop_hold();
  pid_until_dis(1080,1,180,950);
  ballway_mode = 4;
  Move_Stop_hold();

  Turn(74,600);
  PIDGMove(933,70,2000);
  Move_Stop_hold();
  task::sleep(50);
  Turn(-45,650);

  // 7 balls in the low goal ///////////////

  GyroMove(40,165,-45);
  Move_Stop_hold();

  ballway_mode = 2;
  task::sleep(50);
  ballway_mode = 15;
  task::sleep(400);
  ballway_mode = 5;
  task::sleep(950);
  ballway_mode = 9;
  // ballway_mode = 2;
  // task::sleep(300);
  // ballway_mode = 15;
  // task::sleep(800);
  // ballway_mode = 5;
  // task::sleep(1000);
  // ballway_mode = 9;
  task::sleep(1000);
  Move(18,18);
  task::sleep(200);
  Move_Stop_hold();

  GyroMove(50,-490,-45);
  ballway_mode = 1;
  GyroMove(20,-50,-45);
  Move_Stop_hold();

  Turn(-103,500);

  //ballway_mode = 4;
  PIDGMove(3500,-97,3000);
  Turn(-180,450);

  // Clear third loader //////// 
  Load.set(1);
  ballway_mode =4;
  Move_to_blue(180,65);
  Move(45,45);
  task::sleep(400);
  Move(20,20);
  task::sleep(1700);
  GyroMove(65,-320,-240);

  Move_Stop_brake();
  GyroMove(60,-300,-179);
  GyroMove(100,-3200,-179);
  Load.set(0);
  GyroMove(60,-400,-110);
  GyroMove(30,-300,-90);
  GyroMove(35,-200,-90);
  Move_Stop_brake();

  Turn(-10,350);
  Move_to_yellow(-300,60);
  Move(-40,-40);
  task::sleep(250);
  Move_Stop_coast();
  ballway_mode = 21;
  task::sleep(250);
  ballway_mode = 1;
  task::sleep(1300);

  ballway_mode = 4; // Clear fourth loader
  Load.set(1);
  Move_to_red(300,60);
  Move_to_red(350,30);
  Move(45,45);
  task::sleep(300);
  Move(20,20);
  task::sleep(1700);
  Move_to_yellow(-600,70);
  Move_to_yellow(-170,30);
  Move(-30,-30);
  task::sleep(200);
  Move_Stop_coast();
  ballway_mode = 1;
  task::sleep(1250);
  Load.set(0);
  
  GyroMove(100,2000,70);
  GyroMove(100,700,90);
  Move_Stop_brake();


}

/*
void pidTurn_autoTuner(){
  timer t;
  int elapsed_time = 0;
  float score = 0;
  float best_score = 10000;
  float best_kp = 0;
  float best_kd = 0;
  for(float j = 0; j < 20; j+=5){
    for(float i = 0; i < 40; i+=5){
      t.reset();
      score = Turn(90,i,j);
      elapsed_time = t.time();
      printf("kp: %d, ki: %d, time: %d\n", 1 + 0.1 * i, 1+ 0.1 * j, elapsed_time);
      printf("Evaluation: %d\n", score);
      if(score < best_score){
        best_score = score;
        best_kp = 1 + 0.1 * i;
        best_kd = 1 + 0.1 * j;
      }
      task::sleep(1000);
    }
  }
  printf("Best Evaluation: %d, Best parameters: kp-%d kd-%d\n", best_score, best_kp, best_kd);
}
*/

void thread_Move()
{
  int Axis_1,Axis_3;
  while (1) 
  {
    Axis_1 = Controller1.Axis1.value();
    Axis_3 = Controller1.Axis3.value();

    if(fabs(Axis_1) > 1 || fabs(Axis_3) > 1){
      Move(Axis_3 + Axis_1, Axis_3 - Axis_1);
    }
    else{
      Move_Stop_coast();
    }
  }
}

bool Button_A,Button_Y;
bool Button_B,Button_X;
bool Button_L1,Button_L2;
bool Button_R1,Button_R2;
bool Button_Up,Button_Down,Button_Left,Button_Right;
//bool ring = 1;
void drivercontrol(void) 
{
  thread move = thread(thread_Move);
  //thread ball_selection = thread(select);
  //bool loading = 0;
  //bool stick_out = 0;
  int b = 0;
  //thread_ring = 1;
  while (1) 
  {
    Button_A=Controller1.ButtonA.pressing();
    Button_Y=Controller1.ButtonY.pressing();
    Button_B=Controller1.ButtonB.pressing();
    Button_X=Controller1.ButtonX.pressing();
    Button_L1=Controller1.ButtonL1.pressing();    
    Button_L2=Controller1.ButtonL2.pressing();
    Button_R1=Controller1.ButtonR1.pressing();
    Button_R2=Controller1.ButtonR2.pressing();
    Button_Up=Controller1.ButtonUp.pressing();
    Button_Down=Controller1.ButtonDown.pressing();
    Button_Left=Controller1.ButtonLeft.pressing();
    Button_Right=Controller1.ButtonRight.pressing();

    // Intake control
    if(Button_R1)ballway_mode = 4;//吸球
    else if(Button_L1) ballway_mode = 1;//发射高筒
    else if(Button_L2) ballway_mode = 3;//发射中桶 
    else if(Button_R2) ballway_mode = 2;//吐球
    else ballway_mode = 0;//停止

    // Load control
    if(Load.value() == 0 && Controller1.ButtonY.pressing()){
       Controller1.rumble(".");
      Load.set(1);
      waitUntil(!Controller1.ButtonY.pressing());
    }
    else if(Load.value() == 1 && Controller1.ButtonY.pressing()){
      Controller1.rumble("-");
      Load.set(0);
      waitUntil(!Controller1.ButtonY.pressing());
    }

    // Anti-longGoal Control
    if(Anti_LongGoal.value() == 0 && Controller1.ButtonRight.pressing()){
      Controller1.rumble("..");
      Anti_LongGoal.set(1);
      waitUntil(!Controller1.ButtonRight.pressing());
    }
    else if(Anti_LongGoal.value() == 1 && Controller1.ButtonRight.pressing()){
      Controller1.rumble("--");
      Anti_LongGoal.set(0);
      waitUntil(!Controller1.ButtonRight.pressing());
    }

    // Double Park
    if(double_park.value() == 1 && Controller1.ButtonB.pressing()){
      Controller1.rumble("...");
      double_park.set(0);
      waitUntil(!Controller1.ButtonB.pressing());
    }
    else if(double_park.value() == 0 && Controller1.ButtonB.pressing()){
      Controller1.rumble("---");
      double_park.set(1);
      waitUntil(!Controller1.ButtonB.pressing());
    }

    // Head Control
    if(Head.value() == 1 && Controller1.ButtonDown.pressing()){
      Controller1.rumble(".-.");
      Head.set(0);
      waitUntil(!Controller1.ButtonDown.pressing());
    }
    else if(Head.value() == 0 && Controller1.ButtonDown.pressing()){
      Controller1.rumble("-.-");
      Head.set(1);
      waitUntil(!Controller1.ButtonDown.pressing());
    }
    
    /*
    // beta: PID Turn Auto-tuner
    if(Controller1.ButtonUp.pressing()){
      waitUntil(!Controller1.ButtonUp.pressing());
      pidTurn_autoTuner();
    }
    */
    task::sleep(20);
  }
}

int main() 
{
  Optical_sensor.setLightPower(100);
  thread UPDATE = thread(updates);
  task task_ballway = task(ballway_progress,task::taskPriorityHigh);

  Competition.drivercontrol(drivercontrol);
  pre_auton();
  Competition.autonomous(autonomous);
  while(1)
  {
  task::sleep(10);
  }
}
