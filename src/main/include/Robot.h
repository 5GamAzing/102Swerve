/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/PWMTalonSRX.h>
#include <frc/XboxController.h>
#include <frc/Encoder.h>
#include <frc/GenericHID.h>

//Shortcuts
#define LHand frc::GenericHID::kLeftHand
#define RHand frc::GenericHID::kRightHand

class Robot : public frc::TimedRobot {
  public:
    Robot();
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void testHall();
    void swerveDrive(int mode);
    void moveMotors(int *targetEncoders, float *targetSpeeds, int ENCODERS, int SPEED);

    //Input controller definitions
    frc::XboxController *control1;
    frc::XboxController *control2;

    //Motor definitions
    frc::PWMTalonSRX *flMain;
    frc::PWMTalonSRX *frMain;
    frc::PWMTalonSRX *brMain;
    frc::PWMTalonSRX *blMain;
    frc::PWMTalonSRX *flTurn;
    frc::PWMTalonSRX *frTurn;
    frc::PWMTalonSRX *brTurn;
    frc::PWMTalonSRX *blTurn;

    //Sensor definitions
    frc::Encoder *flHall;
    frc::Encoder *frHall;
    frc::Encoder *brHall;
    frc::Encoder *blHall;

    //Encoder variables
    int flEnc, frEnc, brEnc, blEnc;
    int flOffset, frOffset, brOffset, blOffset;

  private:
    //No idea what this is
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;
};