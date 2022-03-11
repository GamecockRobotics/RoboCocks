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
// driveFrontLeft       motor         20              
// driveMiddleLeft      motor         3               
// driveBackLeft        motor         2               
// driveFrontRight      motor         11              
// driveMiddleRight     motor         8               
// driveBackRight       motor         9               
// DigitalOutF          digital_out   F               
// Gyro                 inertial      12              
// RightLift            motor         10              
// LeftLift             motor         1               
// DigitalOutH          digital_out   H               
// Intake               motor         6               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;

const int WHEEL_RADIUS = 2;
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
  DigitalOutF.set(true);
  DigitalOutH.set(true);
}

void driveForward(int dist, bool waiting = true) {
  driveFrontLeft.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  driveMiddleLeft.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  driveBackLeft.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  driveFrontRight.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  driveMiddleRight.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  driveBackRight.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, waiting);
}

void backGrab(bool backClawState){
  DigitalOutH.set(backClawState);
}

void frontGrab(bool frontClawState){
  DigitalOutF.set(frontClawState);
}

void lift(bool dir, float dist){
  if(dir){
    RightLift.spinFor(forward,dist,turns);
    LeftLift.spinFor(forward,dist,turns);
  }else if (dir == false){
    RightLift.spinFor(reverse,dist,turns);
    LeftLift.spinFor(reverse,dist,turns);
  } else{
    RightLift.stop(hold);
    LeftLift.stop(hold);
  }
}

void chassisTurn (double deg, turnType dir) {
  float error = deg;
  float prevError = deg;
  float totalError = 0;
  const float threshold = 2.0;
  const float kp = 0.50;
  const float kd = 0.12;
  const float ki = 0.00;
  Gyro.setRotation(0, degrees);
  while (fabs(error) > threshold ||fabs (prevError) > threshold) {
    int speed = kp*error+kd*(prevError-error) + ki*totalError;
    driveFrontRight.spin(dir == right? forward:reverse, speed, percent);
    driveMiddleRight.spin(dir == right? forward:reverse, speed, percent);
    driveBackRight.spin(dir == right? forward:reverse, speed, percent);
    driveFrontLeft.spin(dir == left?forward:reverse, speed, percent);
    driveMiddleLeft.spin(dir == left?forward:reverse, speed, percent);
    driveBackLeft.spin(dir == left?forward:reverse, speed, percent);
    wait(200, msec);
    prevError = error;
    error = deg - fabs(Gyro.rotation());
    if (fabs(error) < 10) {
      totalError = totalError + error;
    }
  }
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
  /*
  driveForward(47);
  frontGrab(true);
  lift(true, 1);
  driveForward(-47);*/

  //Skills Do not delete
  /*
  frontGrab(false);
  lift(true,1);

  chassisTurn(90,left);
  
  driveForward(42);
  chassisTurn(90,right);
  driveForward(-2);
  backGrab(false);

  driveForward(2);
  chassisTurn(90,right);
  driveForward(22);

  chassisTurn(180,right);
  backGrab(true);
  

  */


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
bool clawState;
bool clawState2;
void frontClaw(){
  if(clawState){
      clawState = false;
      DigitalOutF.set(clawState);
      wait(100, msec);
    } else {
      clawState = true;
      DigitalOutF.set(clawState);
      wait(100, msec);
    }
}

void backClaw(){
  if(clawState2){
      clawState2 = false;
      DigitalOutH.set(clawState2);
      wait(100, msec);
    } else {
      clawState2 = true;
      DigitalOutH.set(clawState2);
      wait(100, msec);
    }
}
 const int speed = 100;

void usercontrol(void) {
  // User control code here, inside the loop
   clawState = true;
   clawState2 = false;
   DigitalOutF.set(clawState);
   DigitalOutH.set(clawState2);
  bool aPressed = false;
  bool bPressed = false;
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    
    driveFrontLeft.spin(forward, -Controller1.Axis3.value(), percent);
    driveMiddleLeft.spin(forward, -Controller1.Axis3.value(), percent);
    driveBackLeft.spin(forward, -Controller1.Axis3.value(), percent);
    driveFrontRight.spin(forward, Controller1.Axis2.value(), percent);
    driveMiddleRight.spin(forward, Controller1.Axis2.value(), percent);
    driveBackRight.spin(forward, Controller1.Axis2.value(), percent);

    //This is to change the claws from toggle to different buttons
    /*
    if(Controller1.ButtonR1.pressing()){
      DigitalOutF.set(true);
    } else if (Controller1.ButtonR2.pressing()){
      DigitalOutF.set(false);
    }

    if(Controller1.ButtonL1.pressing()){
      DigitalOutH.set(true);
    } else if (Controller1.ButtonL2.pressing()){
      DigitalOutH.set(false);
    }*/

    //Claw
    Controller1.ButtonR1.pressed(frontClaw);
    Controller1.ButtonR2.pressed(backClaw);
    
    //Lift
    if(Controller1.ButtonL1.pressing()){
      RightLift.spin(forward, speed, percent);
      LeftLift.spin(forward, speed, percent);
    } else if(Controller1.ButtonL2.pressing()){
      RightLift.spin(reverse, speed, percent);
      LeftLift.spin(reverse, speed, percent);
    } else{
      RightLift.stop(hold);
      LeftLift.stop(hold);
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
