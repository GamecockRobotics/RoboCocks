/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

#ifndef INCLUDE_VEX_H_
#define INCLUDE_VEX_H_
enum intakeDirection { intake, outtake, stopped };
const int WHEEL_RADIUS = 2;

void p();
void driveForward(double dist, bool waiting = true);
void backGrab() ;
void frontGrab();
void turnChassis (double deg, turnType dir);
void leftChassisSpin(int speed, brakeType type = coast) ;
void rightChassisSpin(int speed, brakeType type = coast) ;
void moveArm(bool up, bool down, int speed = 100, brakeType type = hold);
void toggleIntake() ;
void toggleOuttake() ;
void frontClaw(bool close, bool open);
void backClaw(bool close, bool open);
void liftArm(bool waiting = false);

#endif