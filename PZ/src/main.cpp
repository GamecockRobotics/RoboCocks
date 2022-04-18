// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Gyro                 inertial      15              
// LeftChassis0         motor         1               
// LeftClamp            motor         2               
// RightChassis0        motor         9               
// LeftChassis1         motor         12              
// LeftArm              motor         13              
// LeftChassis2         motor         14              
// RightArm             motor         17              
// RightChassis1        motor         18              
// RightChassis2        motor         19              
// RightClamp           motor         10              
// Intake               motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Gyro                 inertial      15              
// LeftChassis0         motor         1               
// LeftClamp            motor         2               
// RightChassis0        motor         9               
// LeftChassis1         motor         12              
// LeftArm              motor         13              
// LeftChassis2         motor         14              
// RightArm             motor         17              
// RightChassis1        motor         18              
// RightChassis2        motor         19              
// RightClamp           motor         10              
// Intake               motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Gyro                 inertial      15              
// LeftChassis0         motor         1               
// LeftClamp            motor         2               
// RightChassis0        motor         9               
// LeftChassis1         motor         12              
// LeftArm              motor         13              
// LeftChassis2         motor         14              
// RightArm             motor         17              
// RightChassis1        motor         18              
// RightChassis2        motor         19              
// RightClamp           motor         10              
// Intake               motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;

const int WHEEL_DIAMETER = 4;
bool clawState;
bool clawState2;
// const float GEAR_DIAMETER = 3.5;
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

void drive (double dist) {
  double errorL = dist/WHEEL_DIAMETER*M_1_PI;
  double errorR = dist/WHEEL_DIAMETER*M_1_PI;
  double errorA = 0;
  double prevErrorL = errorL;
  double prevErrorR = errorR;
  double prevErrorA = 0;
  double totalErrorL = 0;
  double totalErrorR = 0;
  double totalErrorA = 0;
  const double threshold = 2.0;
  const float kp = 0.50;
  const float kd = 0.12;
  const float ki = 0.01;
  const float kpa = 0.05;
  const float kda = 0.01;
  const float kia = 0.001;
  bool left = true;
  bool right = true;
  bool straight = true;
  LeftChassis0.setRotation(0, degrees);
  RightChassis0.setRotation(0, degrees);
  while (left || right || !straight) {
    int speedL = kp*errorL+kd*(prevErrorL-errorL) + ki*totalErrorL;// - (kpa*errorA + kda*(prevErrorA-errorA) +kia*totalErrorA);
    int speedR = kp*errorR+kd*(prevErrorR-errorR) + ki*totalErrorR;// + (kpa*errorA + kda*(prevErrorA-errorA) +kia*totalErrorA);
    if (left) {
      LeftChassis0.spin(forward, speedL, percent);
      LeftChassis1.spin(forward, speedL, percent);
      LeftChassis2.spin(forward, speedL, percent);
    } else {
      LeftChassis0.stop(hold);
      LeftChassis1.stop(hold);
      LeftChassis2.stop(hold);
    }
    if (right) {
      RightChassis0.spin(forward, speedR, percent);
      RightChassis1.spin(forward, speedR, percent);
      RightChassis2.spin(forward, speedR, percent);
    } else {
      RightChassis0.stop(hold);
      RightChassis1.stop(hold);
      RightChassis2.stop(hold);
    }
    wait(200, msec);
    prevErrorL = errorL;
    prevErrorR = errorR;
    prevErrorA = errorA;
    errorL = dist/WHEEL_DIAMETER*M_1_PI - LeftChassis0.rotation(degrees);
    errorR = dist/WHEEL_DIAMETER*M_1_PI - RightChassis0.rotation(degrees);
    errorA = fabs(Gyro.heading()) > 3 ? Gyro.heading() : 0;
    if (fabs(errorL) < 10) {
      totalErrorL = totalErrorL + errorL;
    }
    if (fabs(errorR) < 10) {
      totalErrorR = totalErrorR + errorR;
    }
    totalErrorA = totalErrorA + errorA;
    left = fabs(errorL) > threshold || fabs (prevErrorL) > threshold;
    right = fabs(errorR) > threshold || fabs (prevErrorR) > threshold;
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

void toggleIntake() {
  intakeState = (intakeState == intake ? stopped : intake);
  if (intakeState == intake) {
    Intake.spin(forward, 100, percent);
  } else {
    Intake.stop(coast);
  }
}
void toggleOuttake() {
  intakeState = (intakeState == outtake ? stopped : outtake);
  if (intakeState == outtake) {
    Intake.spin(reverse, 100, percent);
  } else {
    Intake.stop(coast);
  }
}

void leftDrive(double speed) {
  if (fabs(speed) > 5) {
    LeftChassis0.spin(forward, speed, percent);
    LeftChassis1.spin(forward, speed, percent);
    LeftChassis2.spin(forward, speed, percent);
  } else {
    LeftChassis0.stop(coast);
    LeftChassis1.stop(coast);
    LeftChassis2.stop(coast);
  }
}

void rightDrive(double speed) {
  if (fabs(speed) > 5) {
    RightChassis0.spin(forward, speed, percent);
    RightChassis1.spin(forward, speed, percent);
    RightChassis2.spin(forward, speed, percent);
  } else {
    RightChassis0.stop(coast);
    RightChassis1.stop(coast);
    RightChassis2.stop(coast);
  }
}

void usercontrol(void) {
  // User control code here, inside the loop

  
  Controller1.ButtonA.pressed(toggleIntake);
  Controller1.ButtonB.pressed(toggleOuttake);

  while (true) {
    leftDrive(Controller1.Axis3.value());
    rightDrive(Controller1.Axis2.value());

    if (Controller1.ButtonL1.pressing()) {
      LeftClamp.spin(forward, 100, percent);
      RightClamp.spin(forward, 100, percent);
    } else if (Controller1.ButtonL2.pressing()) {
      LeftClamp.spin(reverse, 100, percent);
      RightClamp.spin(reverse, 100, percent);
    } else {
      LeftClamp.stop(brake);
      RightClamp.stop(brake);
    }

    if (Controller1.ButtonR1.pressing()) {
      LeftArm.spin(forward, 100, percent);
      RightArm.spin(forward, 100, percent);
    } else if (Controller1.ButtonR2.pressing()) {
      LeftArm.spin(reverse, 100, percent);
      RightArm.spin(reverse, 100, percent);
    } else {
      LeftArm.stop();
      RightArm.stop();
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.
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
