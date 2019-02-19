/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <wpi/raw_ostream.h>
#include <cameraserver/CameraServer.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/SerialPort.h>
#include <cscore_oo.h>

 const int BAUD = 115200;

/*
class JeVois
{
  const int JEVOIS = 0;
 

  public:
  cs::UsbCamera visionCam;
  frc::SerialPort VisionPort;

  JeVois();
  void connectJeVois();
  void startCameraStream();
};  

JeVois::JeVois()
{
    wpi::errs() << "Jevois \n";
    connectJeVois();
}

void JeVois::connectJeVois()  
{
//    VisionPort = frc::SerialPort(BAUD, frc::SerialPort::Port::kUSB, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One );
    VisionPort = frc::SerialPort(
                                  BAUD, 
                                  frc::SerialPort::Port::kUSB
                             );

}

void JeVois::startCameraStream()
{
   //visionCam = new cs::UsbCamera("visionCam", JEVOIS);
   //visionCam.setVideoMode(PixelFormat.kMJPEG, 320,240,15);
   //camServer = new MjpegServer("visionCam",1180);
   //camServer.SetSource(visionCam);
}

*/


/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot {
  WPI_VictorSPX m_leftFrontMotor{0};
  WPI_VictorSPX m_rightFrontMotor{15};
  WPI_VictorSPX m_leftRearMotor{1};
  WPI_VictorSPX m_rightRearMotor{14};

  TalonSRX m_Tongue{2};

  //JeVois Camera;
  
    const int JEVOIS = 0;
  //m_leftFrontMotor.SetInverted(true);
  //m_rightFrontMotor.SetInverted(true);
  //m_leftRearMotor.SetInverted(true);
  //m_rightRearMotor.SetInverted(true);

  frc::SpeedControllerGroup m_Left{m_leftFrontMotor, m_leftRearMotor};
  frc::SpeedControllerGroup m_Right{m_rightFrontMotor, m_rightRearMotor};
  
  frc::DifferentialDrive m_robotDrive{m_Left, m_Right};
  frc::Joystick m_stick0{0};
  //frc::Joystick m_stick1{1};  
  //frc::Joystick m_stick2{2};  
  //frc::Joystick m_stick3{3};  
  //frc::Joystick m_stick4{4};
  //frc::Joystick m_stick5{5};

 
  frc::DoubleSolenoid m_Ramp{1, 2};
  frc::DoubleSolenoid m_Arm{5,6};
  frc::DoubleSolenoid m_Hatch{3,4};
  //frc::DoubleSolenoid m_Arm{3,4};
  //frc::DoubleSolenoid m_Hatch{5,6};
  //frc::DoubleSolenoid m_Release{7,8};
 
  static constexpr int kRamp_Lower = 2;       //button B
  static constexpr int kRamp_Raise = 1;       //button A
  static constexpr int kRelease_Drop = 5;     //Left Bumper
  static constexpr int kRelease_Retrack = 7;  //Select
  static constexpr int kArm_Up = 8;           //Start
  static constexpr int kArm_Down = 6;         //Right Bumper
  static constexpr int kHatch_Grab = 3;        //Right Trigger
  static constexpr int kHatch_Release = 2;     //Left Trigger

  cs::UsbCamera *visionCam;
  frc::SerialPort VisionPort;
  cs::MjpegServer camServer;

 public:
 void RobotInit() override {
#if defined(__linux__)
    frc::CameraServer::GetInstance()->StartAutomaticCapture();
    //frc::CameraServer::GetInstance()->setVideoMode(PixelFormat.kYUV, 320, 252, 30);		
#else
    wpi::errs() << "Vision only available on Linux.\n";
    wpi::errs().flush();
#endif

visionCam = new cs::UsbCamera("visionCam", JEVOIS);
   //visionCam.setVideoMode(cs::PixelFormat.kMJPEG, 320,240,15);
   camServer = new cs::MjpegServer("visionCamserver",1180);
   camServer.SetSource(visionCam);
  }

  void TeleopPeriodic() {
    m_robotDrive.SetSafetyEnabled(true);
    // Drive with arcade style
    //m_robotDrive.ArcadeDrive(-m_stick0.GetY(), -m_stick0.GetX());
    m_robotDrive.TankDrive(-m_stick0.GetRawAxis(1), -m_stick0.GetRawAxis(5));      //Tank Control Left
    //m_robotDrive.TankDrive(m_stick0.GetX(), m_stick5.GetY());      //Tank Control Right  
    //wpi::errs() << "Teleop joystick" << m_stick.GetY() << "\n";
    //wpi::errs().flush();
    //m_Tongue.Set(ControlMode::PercentOutput, m_stick1.GetY() );
  
  

    /* In order to set the double solenoid, we will say that if neither button
     * is pressed, it is off, if just one button is pressed, set the solenoid to
     * correspond to that button, and if both are pressed, set the solenoid to
     * Forwards.
     */
 if (m_stick0.GetRawButton(kRamp_Lower)) {
      m_Ramp.Set(frc::DoubleSolenoid::kForward);
    } else if (m_stick0.GetRawButton(kRamp_Raise)) {
      m_Ramp.Set(frc::DoubleSolenoid::kReverse);
    } else {
      m_Ramp.Set(frc::DoubleSolenoid::kOff);
    }
/*  if (m_stick0.GetRawButton(kRelease_Drop)) {
      m_Release.Set(frc::DoubleSolenoid::kForward);
    } else if (m_stick0.GetRawButton(kRelease_Retrack)) {
      m_Release.Set(frc::DoubleSolenoid::kReverse);
    } else {
      m_Release.Set(frc::DoubleSolenoid::kOff);
    } */
 if (m_stick0.GetRawButton(kArm_Up)) {
      m_Arm.Set(frc::DoubleSolenoid::kForward);
    } else if (m_stick0.GetRawButton(kArm_Down)) {
      m_Arm.Set(frc::DoubleSolenoid::kReverse);
    } else {
      m_Arm.Set(frc::DoubleSolenoid::kOff);
    }
 if ( m_stick0.GetRawAxis(kHatch_Release) > 0.5) {
      m_Hatch.Set(frc::DoubleSolenoid::kForward);
    } else if ( m_stick0.GetRawAxis(kHatch_Grab) > 0.5) {
      m_Hatch.Set(frc::DoubleSolenoid::kReverse);
    } else {
      m_Hatch.Set(frc::DoubleSolenoid::kOff);
    }
  }
  // if ( m_stick2.GetY() > 0.5) {
  //     m_Hatch.Set(frc::DoubleSolenoid::kForward);
  //   } else if ( m_stick3.GetY() > 0.5) {
  //     m_Hatch.Set(frc::DoubleSolenoid::kReverse);
  //   } else {
  //     m_Hatch.Set(frc::DoubleSolenoid::kOff);
  //   }
  // }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
