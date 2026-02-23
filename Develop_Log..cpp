/*

// ------------- August 25 ------------ //
->Finished the global positioning system for the robot, including desired position coordinates and actually position report.
->Two extra turning functions(partial drivetrain turning using either side of the drivetrain) created for more diversified commands on auton movements.
->A new function proposed: using GyroMove without PID for the first 70%(adjustable) of total moving distance, and combined PID for the rest 30%, 
  making the whole move both fast and precise. 
-> ***A lot of changes to the program need to be tested on a robot***

// ------------- August 29 ------------ //
->Tuned constants for GyroTurn Series, and PIDGMove.
->Found out that turning with only one side of drivetrain will drift a lot, which there is no means to measure currently (drifts).

// ------------- August 31 ------------ //
-> Added(but not tested on bot), implemented Low Pass Filter for heading data coming from inertial sensor.
-> Changed the method for obtaining heading in some functions in PIDG.cpp
-> Heading now is not calculated seeprately in every function, it's continuously updating throughout the auton period and used by functions when needed.
-> Updated arm control part according to feedback from the test drive in August 29.

// ------------- September 1 ------------ //
-> Tested GyroTurn after added LPF, failed. Turning didn't stop at all. Presumed to be problem of target angle caused by constant alpha in LPF
    _) According to ChatGPT, the problem is fixed by eliminating double updates to variable H in getHeading() and pos() two threads.
    _) Waiting to be tested.
-> Added ring selection hardwares, but the codes for it won't work, suspecting misusage of thread in cpp.
-> Updated arm control part according to feedback from the test drive again.
-> Updated claw and whiteflag control to L1 and L2 buttons.

// ------------- September 5 ------------ //
-> Implemented ring selection through air cylinder, and it worked well
-> Adjusted(lowered) turning speed for driver control
-> Adjusted(Higher) aixs threashold for raising/lowering arm

// ------------- September 8 ------------ //
-> Added slow start and slow end for GyroMove(), presumably increasing stability of the movement.
-> Added ring selection to auton

// ------------- September 9 ------------ //
-> Programmed a four-ring auton(Unfinished 6+1 ring)

// ------------- September 12 ------------ //
-> Added a new parameter to GyroTurn(), condition of ending a piece of turning "endV", so that higher efficiency can 
   be achieved facing different turning conditions.
-> Speeding up the 5-ring auton, but caused it to perform less stably
-> Changed the "no ring selection" mode during driver control period: red ring can only be raised to the end of the 
   conveyor and cannot be thrown out, waiting to be backed into the basket
-> Created a new moving function FuzzyGyroMove(), using fuzzy logic control to make the speeding up and slowing down 
   phases less harsh compared with previous GyroMove(). Mainly used in high speed GyroMove
-> All intervals between moving and turning should contain a Move_stop_hold() to enhance consistency
-> Changed the arm's idle state to coast, making it less stiff when scoring on wall stakes
-> CHanged drivtrain's idle status to brake, reduce the chance of drivetrain over-heating.
























*/