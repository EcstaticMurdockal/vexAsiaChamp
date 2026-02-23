#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Left_Motor1(PORT13, gearSetting::ratio6_1,1);
motor Left_Motor2(PORT12, gearSetting::ratio6_1,1);
motor Left_Motor3(PORT11, gearSetting::ratio6_1,0);
motor Right_Motor1(PORT20, gearSetting::ratio6_1,0);
motor Right_Motor2(PORT19, gearSetting::ratio6_1,0);
motor Right_Motor3(PORT17, gearSetting::ratio6_1,1);

rotation d_left = rotation(PORT4,0);
rotation d_right = rotation(PORT6,1);

distance dis_front(PORT21);
distance dis_back(PORT15);
distance Ballway_Distance_Mid(PORT1);
distance Ballway_Distance_Up(PORT10);

motor intake1(PORT9, gearSetting::ratio6_1,0);
motor intake2(PORT5, gearSetting::ratio6_1,1);
motor intake3(PORT3, gearSetting::ratio6_1,0);
motor intake4(PORT7, gearSetting::ratio6_1,0);
motor Intake_group[4] = {intake1,intake2,intake3,intake4};  

optical Optical_sensor(PORT8);

digital_out Head(Brain.ThreeWirePort.C);
digital_out Load(Brain.ThreeWirePort.A);
digital_out double_park(Brain.ThreeWirePort.B);
digital_out Anti_LongGoal(Brain.ThreeWirePort.B);

inertial Inertial_1(PORT16);
//inertial Inertial_2(PORT5);

/*vex-vision-config:begin*/
signature Vision__SIG_1 = signature (1, 5101, 5957, 5530, -6301, -5901, -6102, 4.4, 0); // Yellow
signature Vision__SIG_2 = signature (2, 5657, 15019, 10338, -1391, -279, -834, 1.7, 0); // Red
signature Vision__SIG_3 = signature (3, -4303, -3499, -3900, 7213, 10347, 8780, 3, 0); // Blue
vision Vision_back = vision (PORT14, 50, Vision__SIG_1, Vision__SIG_2, Vision__SIG_3);
vision Vision_front = vision (PORT18, 50, Vision__SIG_1, Vision__SIG_2, Vision__SIG_3);
/*vex-vision-config:end*/


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}