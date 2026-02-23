using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;

extern motor Left_Motor1;
extern motor Left_Motor2;
extern motor Left_Motor3;
extern motor Right_Motor1;
extern motor Right_Motor2;
extern motor Right_Motor3;

extern motor intake1;
extern motor intake2;
extern motor intake3;
extern motor intake4;

extern optical Optical_sensor;

extern rotation Rotation;

extern inertial Inertial_1;
extern inertial Inertial_2;

extern rotation d_left;
extern rotation d_right;

extern distance dis_front;
extern distance dis_back;
extern distance Ballway_Distance_Mid;
extern distance Ballway_Distance_Up;

extern digital_out Head;
extern digital_out Load;
extern digital_out double_park;
extern digital_out Anti_LongGoal;

extern vision Vision_back;
extern vision Vision_front;
extern signature Vision__SIG_1;
extern signature Vision__SIG_2;
extern signature Vision__SIG_3;

char get_color();
extern motor Intake_group[4];
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );