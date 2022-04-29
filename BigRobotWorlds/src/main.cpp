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
// Controller1          controller
// driveFrontLeft       motor         16
// driveMiddleLeft      motor         18
// driveBackLeft        motor         19
// driveFrontRight      motor         2
// driveMiddleRight     motor         7
// driveBackRight       motor         5
// Gyro                 inertial      9
// backClawLeft         motor         17
// backClawRight        motor         8
// frontLiftRight       motor         4
// frontLiftLeft        motor         15
// Intake               motor         1
// claw                 digital_out   H
// rightPiston          digital_out   B
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;

const double WHEEL_DIAMETER = 4.0;
const double CHASSIS_GEAR_RATIO = 60.0 / 84.0;
const double kpNoMiddle = 0.50;
const double kdNoMiddle = 0.12;
const double kiNoMiddle = 0.00;
const double kpMiddle = 0.40;
const double kdMiddle = 0.12;
const double kiMiddle = 0.00;

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
  clawState = false;
  clawState2 = false;
  // clawState2 = true;
  claw.set(clawState);
  driveFrontLeft.setVelocity(100, percent);
  driveMiddleLeft.setVelocity(100, percent);
  driveBackLeft.setVelocity(100, percent);
  driveFrontRight.setVelocity(100, percent);
  driveMiddleRight.setVelocity(100, percent);
  driveBackRight.setVelocity(100, percent);

  backClawLeft.setVelocity(100, percent);
  backClawRight.setVelocity(100, percent);

  driveFrontLeft.setStopping(coast);
  driveMiddleLeft.setStopping(coast);
  driveBackLeft.setStopping(coast);
  driveFrontRight.setStopping(coast);
  driveMiddleRight.setStopping(coast);
  driveBackRight.setStopping(coast);
}

void driveForward(int dist, bool waiting = true) {

  driveFrontLeft.spinFor(dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360, degrees, false);
  driveMiddleLeft.spinFor(dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360, degrees, false);
  driveBackLeft.spinFor(dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360, degrees, false);
  driveFrontRight.spinFor(dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360, degrees, false);
  driveMiddleRight.spinFor(dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360, degrees, false);
  driveBackRight.spinFor(dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360, degrees, waiting);
}

void frontClaw() {
  if (clawState) {
    clawState = false;
    claw.set(clawState);
    wait(100, msec);
  } else {
    clawState = true;
    claw.set(clawState);
    wait(100, msec);
  }
}

void backClaw(){
  if (clawState2) {
    clawState2 = false;
    backClawLeft.spinFor(-800, degrees);
    backClawRight.spinFor(-800,degrees);
    wait(100, msec);
  } else {
    clawState2 = true;
    backClawLeft.spinFor(800, degrees);
    backClawRight.spinFor(800,degrees);
    wait(100, msec);
    Brain.Screen.clearScreen();
    Brain.Screen.print("Finished Claw");
  }

}

void lift(float ang, bool waiting = false) {
  frontLiftRight.spinFor(ang / 7, turns, false);
  frontLiftLeft.spinFor(ang / 7, turns, waiting);
}

void drive(double dist) {
  double errorL = dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 345;
  double errorR = dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 345;
  // Brain.Screen.print(errorL);
  // Brain.Screen.newLine();
  double prevErrorL = errorL;
  double prevErrorR = errorR;
  double totalErrorL = 0;
  double totalErrorR = 0;
  const double threshold = 16.0;
  const float kp = 0.05;
  const float kd = 0.15;
  const float ki = 0.0001;
  bool left = true;
  bool right = true;
  driveBackLeft.setRotation(0, degrees);
  driveBackRight.setRotation(0, degrees);
  while (left || right) {
    int speedL = kp * errorL + kd * (prevErrorL - errorL) + ki * totalErrorL;
    int speedR = kp * errorR + kd * (prevErrorR - errorR) + ki * totalErrorR;
    if (left) {
      driveBackLeft.spin(forward, speedL, percent);
      driveMiddleLeft.spin(forward, speedL, percent);
      driveFrontLeft.spin(forward, speedL, percent);
    } else {
      driveBackLeft.stop(hold);
      driveMiddleLeft.stop(hold);
      driveFrontLeft.stop(hold);
    }
    if (right) {
      driveBackRight.spin(forward, speedR, percent);
      driveMiddleRight.spin(forward, speedR, percent);
      driveFrontRight.spin(forward, speedR, percent);
    } else {
      driveBackRight.stop(hold);
      driveMiddleRight.stop(hold);
      driveFrontRight.stop(hold);
    }
    wait(200, msec);
    prevErrorL = errorL;
    prevErrorR = errorR;
    errorL = dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 345 -
             driveBackLeft.rotation(degrees);
    errorR = dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 345 -
             driveBackRight.rotation(degrees);
    totalErrorL = totalErrorL + ((fabs(errorL) < 10) ? errorL : 0);
    totalErrorR = totalErrorR + ((fabs(errorR) < 10) ? errorR : 0);
    left = fabs(errorL) > threshold || fabs(prevErrorL) > threshold;
    right = fabs(errorR) > threshold || fabs(prevErrorR) > threshold;
    Brain.Screen.print(errorL);
    Brain.Screen.newLine();
    Brain.Screen.print(errorR);
    
  }
  Brain.Screen.print("Im OUTTTTTTTTTTTTTTTTTTTTTTTTT");
  driveFrontRight.stop();
  driveMiddleRight.stop();
  driveBackRight.stop();
  driveFrontLeft.stop();
  driveMiddleLeft.stop();
  driveBackLeft.stop();
}

void chassisTurn(double deg, turnType dir, const double tkp, const double tkd, const double tki) {
  float error = deg;
  float prevError = deg;
  float totalError = 0;
  const float threshold = 2.0;
  const float kp = tkp; // 0.5
  const float kd = tkd; // 0.12
  const float ki = tki;
  Gyro.setRotation(0, degrees);
  while (fabs(error) > threshold || fabs(prevError) > threshold) {
    int speed = kp * error + kd * (prevError - error) + ki * totalError;
    driveFrontRight.spin(dir == left ? forward : reverse, speed, percent);
    driveMiddleRight.spin(dir == left ? forward : reverse, speed, percent);
    driveBackRight.spin(dir == left ? forward : reverse, speed, percent);
    driveFrontLeft.spin(dir == right ? forward : reverse, speed, percent);
    driveMiddleLeft.spin(dir == right ? forward : reverse, speed, percent);
    driveBackLeft.spin(dir == right ? forward : reverse, speed, percent);
    wait(200, msec);
    prevError = error;
    error = deg - fabs(Gyro.rotation());
    if (fabs(error) < 10) {
      totalError = totalError + error;
    }
  }

  driveFrontRight.stop();
  driveMiddleRight.stop();
  driveBackRight.stop();
  driveFrontLeft.stop();
  driveMiddleLeft.stop();
  driveBackLeft.stop();
}

void setMotorSpeed(int s) {
  driveFrontLeft.setVelocity(s, percent);
  driveMiddleLeft.setVelocity(s, percent);
  driveBackLeft.setVelocity(s, percent);
  driveFrontRight.setVelocity(s, percent);
  driveMiddleRight.setVelocity(s, percent);
  driveBackRight.setVelocity(s, percent);
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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  
  drive(72);
  frontClaw();
  lift(3);
  drive(-44);
  lift(-3, true);
  chassisTurn(45, left, kpMiddle, kdMiddle, kiMiddle);
  driveForward(-10);
  backClaw(); //Test function
  drive(5);
  chassisTurn(90, left, kpMiddle, kdMiddle, kiMiddle);
  driveForward(10);
  setMotorSpeed(30);
  lift(5,true);
  Intake.spin(forward, 80, percent);
  // for(int i = 0; i < 3; i++){
    drive(-12);
    wait(100, msec);
    drive(12);
  // }
  Intake.stop(coast);
  setMotorSpeed(100);


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
    Intake.spin(forward, 80, percent);
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

void leftChassisSpin(int speed) {
  if (abs(speed) > 5) {
    driveBackLeft.spin(forward, speed, percent);
    driveMiddleLeft.spin(forward, speed, percent);
    driveFrontLeft.spin(forward, speed, percent);
  } else {
    driveBackLeft.stop(coast);
    driveMiddleLeft.stop(coast);
    driveFrontLeft.stop(coast);
  }
}
void rightChassisSpin(int speed) {
  if (abs(speed) > 5) {
    driveBackRight.spin(forward, speed, percent);
    driveMiddleRight.spin(forward, speed, percent);
    driveFrontRight.spin(forward, speed, percent);
  } else {
    driveBackRight.stop(coast);
    driveMiddleRight.stop(coast);
    driveFrontRight.stop(coast);
  }
}

const int speed = 100;

void usercontrol(void) {
  // User control code here, inside the loop
  bool aPressed = false;
  bool bPressed = false;
  Controller1.ButtonR1.pressed(frontClaw);
  while (1) {
    leftChassisSpin(Controller1.Axis3.value());
    rightChassisSpin(Controller1.Axis2.value());

    // Claw
    if (Controller1.ButtonLeft.pressing()) {
      backClawLeft.spin(reverse, 100, percent);
      backClawRight.spin(reverse, 100, percent);
    } else if (Controller1.ButtonUp.pressing()) {
      backClawLeft.spin(forward, 100, percent);
      backClawRight.spin(forward, 100, percent);
    } else {
      backClawLeft.stop(hold);
      backClawRight.stop(hold);
    }

    // Lift
    if (Controller1.ButtonL1.pressing()) {
      frontLiftRight.spin(forward, speed, percent);
      frontLiftLeft.spin(forward, speed, percent);
    } else if (Controller1.ButtonL2.pressing()) {
      frontLiftRight.spin(reverse, speed, percent);
      frontLiftLeft.spin(reverse, speed, percent);
    } else {
      frontLiftRight.stop(hold);
      frontLiftLeft.stop(hold);
    }

    // Control for the Intake
    if (Controller1.ButtonA.pressing()) {
      if (!aPressed) {
        toggleIntake();
      }
      aPressed = true;
      bPressed = false;
    } else if (Controller1.ButtonB.pressing()) {
      if (!bPressed) {
        toggleOuttake();
      }
      aPressed = false;
      bPressed = true;
    } else {
      aPressed = false;
      bPressed = false;
    }

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
