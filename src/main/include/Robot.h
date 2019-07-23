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
    //IDK
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void testHall();
    void swerveDrive(int mode);

    //Input controller definitions
    frc::XboxController *control1 = new frc::XboxController(0);
    frc::XboxController *control2 = new frc::XboxController(1);

    //Motor definitions
    frc::PWMTalonSRX *flMain = new frc::PWMTalonSRX(0);
    frc::PWMTalonSRX *frMain = new frc::PWMTalonSRX(2);
    frc::PWMTalonSRX *brMain = new frc::PWMTalonSRX(7);
    frc::PWMTalonSRX *blMain = new frc::PWMTalonSRX(5);
    frc::PWMTalonSRX *flTurn = new frc::PWMTalonSRX(1);
    frc::PWMTalonSRX *frTurn = new frc::PWMTalonSRX(3);
    frc::PWMTalonSRX *brTurn = new frc::PWMTalonSRX(6);
    frc::PWMTalonSRX *blTurn = new frc::PWMTalonSRX(4);

    //Sensor definitions
    frc::Encoder *flHall = new frc::Encoder(0, 1);
    frc::Encoder *frHall = new frc::Encoder(6, 7);
    frc::Encoder *brHall = new frc::Encoder(4, 5);
    frc::Encoder *blHall = new frc::Encoder(2, 3);

  private:
    //No idea what this is
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;
};