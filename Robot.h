// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <ctre/Phoenix.h>
#include "rev/CANSparkMax.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>
#include "frc/DigitalInput.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryParameterizer.h>
#include <frc/controller/RamseteController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/AnalogInput.h>
#include <frc/AnalogOutput.h>

class Robot : public frc::TimedRobot
{
  frc::Compressor Compressor1{30, frc::PneumaticsModuleType::CTREPCM};
  frc::Solenoid WheelStopper{30, frc::PneumaticsModuleType::CTREPCM, 0};
  frc::Solenoid ClimbRachet{30, frc::PneumaticsModuleType::CTREPCM, 2};
  frc::DoubleSolenoid IntakePosition{30, frc::PneumaticsModuleType::CTREPCM, 1, 4};


  TalonFX RightDrive1 {0}; 
  TalonFX RightDrive2 {1}; 
  TalonFX LeftDrive1 {2}; 
  TalonFX LeftDrive2 {3}; 
  PigeonIMU Pigeon {4}; 

  rev::CANSparkMax TopShooter = rev::CANSparkMax(5, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxPIDController TopShooterPID = TopShooter.GetPIDController();
    rev::SparkMaxRelativeEncoder TopShooterEncoder = TopShooter.GetEncoder();

  rev::CANSparkMax BottomShooter = rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxPIDController BottomShooterPID = BottomShooter.GetPIDController();
    rev::SparkMaxRelativeEncoder BottomShooterEncoder = BottomShooter.GetEncoder();

  TalonFX ShooterAngle {7};

  rev::CANSparkMax Magazine = rev::CANSparkMax(8, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxPIDController MagazinePID = Magazine.GetPIDController();

  TalonSRX ClimberAngle1 {10}; 
  TalonSRX ClimberAngle2 {11};

  rev::CANSparkMax Climber1 = rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless); 
    rev::SparkMaxRelativeEncoder Climber1Encoder = Climber1.GetEncoder();
    rev::SparkMaxPIDController Climber1PID = Climber1.GetPIDController();

  rev::CANSparkMax Climber2 = rev::CANSparkMax(13, rev::CANSparkMax::MotorType::kBrushless); 
    rev::SparkMaxRelativeEncoder Climber2Encoder = Climber2.GetEncoder();
    rev::SparkMaxPIDController Climber2PID = Climber2.GetPIDController();

  VictorSPX Intake1 {14};
  
  //frc::DigitalInput TestSwitch = frc::DigitalInput(0); //Limit Switch
  frc::DigitalInput BackSwitch = frc::DigitalInput(1);
  frc::DigitalInput SideSwitch = frc::DigitalInput(0);
  frc::AnalogInput Bottom1 = frc::AnalogInput(0);
  frc::AnalogInput Bottom2 = frc::AnalogInput(1);
   
 public:
  
  PigeonIMU::FusionStatus Status;

  frc::DifferentialDriveOdometry m_odometry = frc::DifferentialDriveOdometry(PigeonToRotation(0));
  frc::DifferentialDriveKinematics DriveKin = frc::DifferentialDriveKinematics(units::length::inch_t(24.7));

  frc::RamseteController PathFollower;
  frc::RamseteController PathFollower2;
  frc::RamseteController PathFollower3;
  frc::RamseteController PathFollower4;
  frc::RamseteController PathFollower5;
  frc::RamseteController PathFollower6;
  frc::RamseteController PathFollower7;
  frc::RamseteController PathFollower8;
  frc::RamseteController PathFollower9;
  frc::RamseteController PathFollower10;
  /*frc::RamseteController PathFollower11;
  frc::RamseteController PathFollower12;
  frc::RamseteController PathFollower13;
  frc::RamseteController PathFollower14;
  frc::RamseteController PathFollower15;
  frc::RamseteController Pathfollower16;*/

  frc::Trajectory trajectory;
  frc::Trajectory trajectory2;
  frc::Trajectory trajectory3; 
  frc::Trajectory trajectory4;
  frc::Trajectory trajectory5;
  frc::Trajectory trajectory6;
  frc::Trajectory trajectory7;
  frc::Trajectory trajectory8;
  frc::Trajectory trajectory9;
  frc::Trajectory trajectory10;
  /*frc::Trajectory trajectory11;
  frc::Trajectory trajectory12;
  frc::Trajectory trajectory13;
  frc::Trajectory trajectory14;
  frc::Trajectory trajectory15;
  frc::Trajectory trajectory16;*/

  frc::Timer Practice = frc::Timer();
  frc::Timer PathTime = frc::Timer();
  frc::Timer Climber = frc::Timer();
  frc::Timer Intaking = frc::Timer();

  units::second_t time_to = units::second_t(5);

  frc::XboxController Driver = frc::XboxController (0);
  frc::XboxController Manipulator = frc::XboxController (1);

  double IntakeSpeed = 2000;
  double Lower = 1000; 
  double Higher = 2600;
  bool StartMath = false;
  int A = 5; //2 ball 
  int B = 10; //3 ball
  int C = 10; //4 ball
  int D = 10; //5 ball
  int Hang = 0;
  int Shooting = 10;
  
 Robot()
 {
    Pigeon.SetFusedHeading(0,30);
    m_odometry = frc::DifferentialDriveOdometry(PigeonToRotation(Pigeon.GetFusedHeading()));
 }
  
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void AutoTwoBall();
  void AutoThreeBall();
  void AutoFourBall();
  void AutoFiveBall();

  frc::Trajectory GenerateTrajectory();
  frc::Trajectory GenerateTrajectory2();
  frc::Trajectory GenerateTrajectory3();
  frc::Trajectory GenerateTrajectory4();
  frc::Trajectory GenerateTrajectory5();
  frc::Trajectory GenerateTrajectory6();
  frc::Trajectory GenerateTrajectory7();
  frc::Trajectory GenerateTrajectory8();
  frc::Trajectory GenerateTrajectory9();
  frc::Trajectory GenerateTrajectory10();
  /*frc::Trajectory GenerateTrajectory11();
  frc::Trajectory GenerateTrajectory12();
  frc::Trajectory GenerateTrajectory13();
  frc::Trajectory GenerateTrajectory14();
  frc::Trajectory GenerateTrajectory15();
  frc::Trajectory GenerateTrajectory16();*/

  double ConversionToRaw(units::meters_per_second_t MSec)
  {
    return MSec.value() * 4418.1126;
  } 
  double ClicksToInch(double clicks) 
  {
    return (1/1122.2) * clicks;
  }
  double InchesToClicks(double inches)
  {
    return (1122.2 * inches);
  }
  frc::Rotation2d PigeonToRotation(double degrees)
  {
    units::angle::degree_t T = units::angle::degree_t(degrees);
    frc::Rotation2d Rotation = frc::Rotation2d(T);
    return Rotation;
  }

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kThreeBall = "Three Ball";
  const std::string kTwoBall = "Two Ball";
  const std::string kFiveBall = "Five Ball";
  const std::string kFourBall = "Four Ball";
  std::string m_autoSelected;
};
