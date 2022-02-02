/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// leftChassis          motor         1               
// rightChassis         motor         10              
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
const double chassisWidth = 59/4;
const double wheelDiameter = 4;
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/




void drive(directionType dir, int dist) {
  rightChassis.spinFor(dir, dist/(M_PI*8), turns);
  leftChassis.spinFor(dir, dist/(M_PI*8), turns);
}

void turn(turnType dir, int radius, double radians) {
  double left_turns = (radius + (dir == left ? -chassisWidth/2 : chassisWidth/2))*radians/(M_PI*wheelDiameter);
  double right_turns = (radius + (dir == right ? -chassisWidth/2 : chassisWidth/2))*radians/(M_PI*wheelDiameter);
  if (left_turns == 0) {
    rightChassis.setVelocity(100, percent);
    leftChassis.setVelocity(0,percent);
  } else if (right_turns == 0) {
    rightChassis.setVelocity(0, percent);
    leftChassis.setVelocity(100, percent);
  } else if (fabs(left_turns) < fabs(right_turns)) {
    rightChassis.setVelocity(100, percent);
    leftChassis.setVelocity(fabs(left_turns)/fabs(right_turns), percent);
  } else {
    rightChassis.setVelocity(fabs(right_turns)/fabs(left_turns), percent);
    leftChassis.setVelocity(100, percent);
  }

}


void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
 
  while (true) {
    leftChassis.spin(forward, Controller1.Axis2.value(), percent);
    rightChassis.spin(forward, Controller1.Axis3.value(), percent);
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
