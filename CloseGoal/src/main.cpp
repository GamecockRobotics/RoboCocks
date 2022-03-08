
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <string>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

const double chassisWidth = 59/4;
const double wheelDiameter = 4;

enum liftType{up, down};
const int upPos = -90;
const int downPos = -120;

const distanceUnits millimeters = distanceUnits::mm;
const distanceUnits centimeters = distanceUnits::cm;
const double convertIN = 1;
const double convertCM = 2.54;
const double convertMM = 25.4;

const rotationUnits radians = rotationUnits::raw;
const rotationUnits revolutions = rotationUnits::rev;
const double convertRad = 2*M_PI;
const double convertDeg = 360;
const double convertRev = 1;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  ArmMotor1.resetPosition();
  ArmMotor2.resetPosition();
  LeftChassis.setVelocity(100,percent);
  RightChassis.setVelocity(100,percent);
  
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




void drive(directionType dir, int dist, distanceUnits units) {
  double unitNorm = (units == millimeters ? convertMM : (units == centimeters ? convertCM :convertIN));
  RightChassis.spinFor(dir, dist/(M_PI*wheelDiameter*unitNorm), turns, false);
  LeftChassis.spinFor(dir, dist/(M_PI*wheelDiameter*unitNorm), turns, true);
}

void turn(turnType dir,  double arc, rotationUnits units) {
  
  double _turns = chassisWidth*M_PI*(arc / (units == rev ? convertRev : units == radians ? convertRad : convertDeg))/(M_PI*wheelDiameter);
  RightChassis.spinFor(dir == right ? forward:reverse, _turns, turns,false);
  RightChassis1.spinFor(dir == right ? forward:reverse, _turns, turns,false);
  LeftChassis.spinFor(dir == left  ? forward:reverse, _turns, turns,false);
  LeftChassis1.spinFor(dir == left ? forward:reverse, _turns, turns,true);
}


void lift(liftType pos) {
  ArmMotor1.spinToPosition(pos == up? upPos:downPos,degrees, false);
  ArmMotor2.spinToPosition(pos == up? upPos:downPos,degrees, true);
}

void fling() {
  ArmMotor1.setVelocity(70, percent);
  ArmMotor2.setVelocity(70, percent);
  ArmMotor1.spinToPosition(downPos, degrees, false);
  ArmMotor2.spinToPosition(downPos, degrees, false);
  wait(0.2, sec);
  
  ArmMotor1.setVelocity(100, percent);
  ArmMotor2.setVelocity(100, percent);

}


void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

 
 drive(forward, 12, inches);
 fling();
 wait(2, sec);
 drive(forward, 20, inches);
 turn(right, 60, degrees);
 lift(down);
 drive(forward, 20, inches);
 lift(up);
 drive(reverse, 48, inches);
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
    LeftChassis.spin(forward, Controller1.Axis2.value(), percent);
    LeftChassis1.spin(forward, Controller1.Axis2.value(), percent);
    RightChassis.spin(forward, Controller1.Axis3.value(), percent);
    RightChassis1.spin(forward, Controller1.Axis3.value(), percent);
    LeftChassis.spi

    if(Controller1.ButtonR1.pressing()){
      ArmMotor1.spin(forward);
      ArmMotor2.spin(forward);
    } else if(Controller1.ButtonR2.pressing()){
      ArmMotor1.spin(reverse);
      ArmMotor2.spin(reverse);
    } else {
      ArmMotor1.stop(hold);
      ArmMotor2.stop(hold);
    } 

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
    //std::string temp ="suff" << 8;
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
