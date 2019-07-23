/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <math.h>

#include "TestHall.cpp"
#include "JoystickControls.cpp"
#include "SwerveDrive.cpp"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() {
  //Input controller definitions
  control1 = new frc::XboxController(0);
  control2 = new frc::XboxController(1);

  //Motor definitions
  flMain = new frc::PWMTalonSRX(0);
  frMain = new frc::PWMTalonSRX(2);
  brMain = new frc::PWMTalonSRX(7);
  blMain = new frc::PWMTalonSRX(5);
  flTurn = new frc::PWMTalonSRX(1);
  frTurn = new frc::PWMTalonSRX(3);
  brTurn = new frc::PWMTalonSRX(6);
  blTurn = new frc::PWMTalonSRX(4);

  //Sensor definitions
  flHall = new frc::Encoder(0, 1);
  frHall = new frc::Encoder(6, 7);
  brHall = new frc::Encoder(4, 5);
  blHall = new frc::Encoder(2, 3);
}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  testHall();
}

void Robot::TeleopInit() {
  (*flHall).Reset();
  (*frHall).Reset();
  (*brHall).Reset();
  (*blHall).Reset();
}

void Robot::TeleopPeriodic() {
  swerveDrive(4);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
