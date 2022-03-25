// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

void Robot::RobotInit() 
{

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  m_chooser.SetDefaultOption(kThreeBall, kThreeBall);
  m_chooser.AddOption(kTwoBall, kTwoBall);
  m_chooser.AddOption(kFourBall, kFourBall);
  m_chooser.AddOption(kFiveBall, kFiveBall);
  
  Compressor1.Enabled();

  //Follow command
  RightDrive2.Follow(RightDrive1);
  LeftDrive2.Follow(LeftDrive1);

  //Controls Ramp Rate
  RightDrive1.ConfigClosedloopRamp(1);
  LeftDrive1.ConfigClosedloopRamp(1);

    //Sets Current limits
  SupplyCurrentLimitConfiguration Limits = SupplyCurrentLimitConfiguration(true, 20, 30, 1);
  RightDrive1.ConfigSupplyCurrentLimit(Limits);
  RightDrive2.ConfigSupplyCurrentLimit(Limits);
  LeftDrive1.ConfigSupplyCurrentLimit(Limits);
  LeftDrive2.ConfigSupplyCurrentLimit(Limits);

  ShooterAngle.ConfigClosedloopRamp(1);
  ShooterAngle.ConfigPeakOutputReverse(-1);

  //Power Limit
  RightDrive1.ConfigPeakOutputForward(.65);
  LeftDrive1.ConfigPeakOutputForward(.65);

  ShooterAngle.ConfigPeakOutputForward(1);
  ShooterAngle.SetNeutralMode(Brake);

  // ClimberAngle1.ConfigPeakOutputForward(.4);
  // ClimberAngle2.ConfigPeakOutputForward(.4);

  TopShooter.SetClosedLoopRampRate(1.5);
  BottomShooter.SetClosedLoopRampRate(1.5);

  //Defines Trajectory 
  trajectory = GenerateTrajectory();
  trajectory2 = GenerateTrajectory2();
  trajectory3 = GenerateTrajectory3();
  trajectory4 = GenerateTrajectory4();
  trajectory5 = GenerateTrajectory5();
  trajectory6 = GenerateTrajectory6();
  trajectory7 = GenerateTrajectory7();
  trajectory8 = GenerateTrajectory8();
  trajectory9 = GenerateTrajectory9();
  trajectory10 = GenerateTrajectory10();
  /*trajectory11 = GenerateTrajectory11();
  trajectory12 = GenerateTrajectory12();
  trajectory13 = GenerateTrajectory13();
  trajectory14 = GenerateTrajectory14();
  trajectory15 = GenerateTrajectory15();
  trajectory16 = GenerateTrajectory16();*/

  frc::SmartDashboard::PutNumber("NumSetPointsRobotInit", trajectory.TotalTime().value());


  //Configures Defult settings
  Pigeon.ConfigFactoryDefault();
  //rotation
  Pigeon.SetFusedHeading(0, 30);

  // ClimberAngle1.ConfigFactoryDefault();
  // ClimberAngle2.ConfigFactoryDefault();

  // Climber1.RestoreFactoryDefaults();
  // Climber2.RestoreFactoryDefaults();

  TopShooter.RestoreFactoryDefaults();
  BottomShooter.RestoreFactoryDefaults();

  // ShooterAngle.ConfigFactoryDefault();

  Climber2.SetClosedLoopRampRate(.5);
  Climber1.SetClosedLoopRampRate(.5);

  Climber1PID.SetOutputRange(0, .5);
  Climber2PID.SetOutputRange(0, .5);

  Climber1.BurnFlash();
  Climber2.BurnFlash();

  //Min and Max power
  ClimberAngle1PID.SetOutputRange(-1, 1);
  ClimberAngle2PID.SetOutputRange(-1, 1);

  ClimberAngle1.BurnFlash();
  ClimberAngle2.BurnFlash();

  //Setting Inversion
  RightDrive1.SetInverted(true);
  LeftDrive1.SetInverted(false);
  RightDrive2.SetInverted(InvertType::FollowMaster);
  LeftDrive2.SetInverted(InvertType::FollowMaster);

  ClimberAngle1.SetInverted(true);
  ClimberAngle2.SetInverted(true);

  Climber1.SetInverted(true);
  Climber2.SetInverted(true);

  Intake1.SetInverted(false);

  TopShooter.SetInverted(false);
  BottomShooter.SetInverted(true);
  Magazine.SetInverted(true);

  //Defines Drive Sensor
  LeftDrive1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  RightDrive1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

  //Defining Climb encoders
  // ClimberAngle1.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
  // ClimberAngle2.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

  //ShooterAngle.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Signed_PlusMinus180, 10);
  // ShooterAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  
  //Shooter PIDS
  TopShooterPID.SetP(.00006, 0); 
  BottomShooterPID.SetP(.00006, 0);

  TopShooterPID.SetI(.000001, 0);
  BottomShooterPID.SetI(.000001, 0); 

  TopShooterPID.SetD(.03);
  BottomShooterPID.SetD(.03);

  MagazinePID.SetP(.0001);

  TopShooter.BurnFlash();
  BottomShooter.BurnFlash();

  //Drive PIDS
  RightDrive1.Config_kP(0, 0.6);
  LeftDrive1.Config_kP(0, 0.6);
  RightDrive1.Config_kD(0, 100);
  LeftDrive1.Config_kD(0, 100);
  LeftDrive1.Config_kI(0, .0001);
  RightDrive1.Config_kI(0,.0001);

  // ShooterAngle.Config_kP(0, .01); 
  // ShooterAngle.Config_kI(0, .000035); 
  // ShooterAngle.Config_kD(0, .1); 
  // ShooterAngle.Config_kF(0, 0);

  Intake1.Config_kP(0, .1);

  // Climber1PID.SetP(.04, 0);
  // Climber2PID.SetP(.04, 0);

  // Climber1PID.SetI(0, 0);//.0000005
  // Climber2PID.SetI(0, 0);

  // Climber1PID.SetD(0, 0);//.0006
  // Climber2PID.SetD(0, 0);

    ClimberAngle1PID.SetP(0.6, 0);
    ClimberAngle2PID.SetP(0.6, 0);

}
//   //Auto Chooser
//   frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
//   m_chooser.SetDefaultOption(kThreeBall, kThreeBall);
//   m_chooser.AddOption(kTwoBall, kTwoBall);
//   m_chooser.AddOption(kFourBall, kFourBall);
//   m_chooser.AddOption(kFiveBall, kFiveBall);
  
//   //Turns on Compressor
//   Compressor1.Enabled();

//  //Defines Trajectory 
//   trajectory = GenerateTrajectory();
//   trajectory2 = GenerateTrajectory2();
//   trajectory3 = GenerateTrajectory3();
//   trajectory4 = GenerateTrajectory4();
//   trajectory5 = GenerateTrajectory5();
//   trajectory6 = GenerateTrajectory6();
//   trajectory7 = GenerateTrajectory7();
//   trajectory8 = GenerateTrajectory8();
//   trajectory9 = GenerateTrajectory9();
//   trajectory10 = GenerateTrajectory10();
//   /*trajectory11 = GenerateTrajectory11();
//   trajectory12 = GenerateTrajectory12();
//   trajectory13 = GenerateTrajectory13();
//   trajectory14 = GenerateTrajectory14();
//   trajectory15 = GenerateTrajectory15();
//   trajectory16 = GenerateTrajectory16();*/

//   //Puts trajectory on smart Dashboard
//   frc::SmartDashboard::PutNumber("NumSetPointsRobotInit", trajectory.TotalTime().value());

//   //Configures Defult settings
//   Pigeon.ConfigFactoryDefault();

//   //Rotation
//   Pigeon.SetFusedHeading(0, 30);

//   //Follow command
//   RightDrive2.Follow(RightDrive1);
//   LeftDrive2.Follow(LeftDrive1);

//   //Controls Ramp Rate (Auto)
//   RightDrive1.ConfigClosedloopRamp(1);
//   LeftDrive1.ConfigClosedloopRamp(1);

//   //Control Ramp Rate (Teleop)
//   RightDrive1.ConfigOpenloopRamp(.1);
//   LeftDrive1.ConfigOpenloopRamp(.1);

//   //Power Limit
//   RightDrive1.ConfigPeakOutputForward(1);
//   LeftDrive1.ConfigPeakOutputForward(1);

//   //Sets Current limits
//   SupplyCurrentLimitConfiguration Limits = SupplyCurrentLimitConfiguration(true, 20, 30, 1);
//   RightDrive1.ConfigSupplyCurrentLimit(Limits);
//   RightDrive2.ConfigSupplyCurrentLimit(Limits);
//   LeftDrive1.ConfigSupplyCurrentLimit(Limits);
//   LeftDrive2.ConfigSupplyCurrentLimit(Limits);

//   //Setting Inversion
//   RightDrive1.SetInverted(true);
//   LeftDrive1.SetInverted(false);

//   //Follow Comand
//   RightDrive2.SetInverted(InvertType::FollowMaster);
//   LeftDrive2.SetInverted(InvertType::FollowMaster);

//   //Defines Drive Sensor
//   LeftDrive1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
//   RightDrive1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

//   //Drive PIDS
//   RightDrive1.Config_kP(0, 0.6);
//   LeftDrive1.Config_kP(0, 0.6);
//   RightDrive1.Config_kD(0, 100);
//   LeftDrive1.Config_kD(0, 100);
//   LeftDrive1.Config_kI(0, .0001);
//   RightDrive1.Config_kI(0,.0001);

//   //Shooter Defult
//   //ShooterAngle.ConfigFactoryDefault();

//   //Defines Shooter Encoder
//   //ShooterAngle.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Signed_PlusMinus180, 10);
//   //ShooterAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

//   //Ramp rate sensor
//   ShooterAngle.ConfigClosedloopRamp(1);

//   //Back and Forward power
//   ShooterAngle.ConfigPeakOutputReverse(-1);
//   ShooterAngle.ConfigPeakOutputForward(1);

//   //Sets mode to neutral
//   ShooterAngle.SetNeutralMode(Brake);

//   //Shooter Angle PIDs
//   ShooterAngle.Config_kP(0, .1); 
//   ShooterAngle.Config_kI(0, .000035); 
//   ShooterAngle.Config_kD(0, .1); 
//   ShooterAngle.Config_kF(0, 0);

//   //Defines the Shooter encoder to use the shooter angle motor
//   //ShooterAngle.ConfigRemoteFeedbackFilter(ShooterEncoder.GetDeviceNumber(), RemoteSensorSource::RemoteSensorSource_CANCoder, 0);
//   //ShooterAngle.ConfigSelectedFeedbackSensor(RemoteFeedbackDevice::RemoteSensor0);

//   //Restores factory setting
//   // TopShooter.RestoreFactoryDefaults();
//   // BottomShooter.RestoreFactoryDefaults();

//   //Sets Velocity ramp rate
//   TopShooter.SetClosedLoopRampRate(1.5);
//   BottomShooter.SetClosedLoopRampRate(1.5);
    
//   TopShooter.BurnFlash();
//   BottomShooter.BurnFlash();

//   //Sets inversions
//   TopShooter.SetInverted(false);
//   BottomShooter.SetInverted(true);

//   //Shooter PIDS
//   TopShooterPID.SetP(.002, 0); //.00001
//   BottomShooterPID.SetP(.002, 0); //.00001
//   TopShooterPID.SetI(0, 0); //.000001
//   BottomShooterPID.SetI(0, 0); //.000001
//   TopShooterPID.SetD(0, 0); //.03
//   BottomShooterPID.SetD(0, 0); //.03

//   //Puts on Motor Controller

//   //Sets inversion 
//   Magazine.SetInverted(true);

//   //Magazine PID
//   MagazinePID.SetP(.0001);

//   //Sets inversion
//   Intake1.SetInverted(false);

//   //Intake PID
//   Intake1.Config_kP(0, .1);

//   //Resotres Factory defults
//   //ClimberAngle1.RestoreFactoryDefaults();
//   //ClimberAngle2.RestoreFactoryDefaults();

//   //Min and Max power
//   ClimberAngle1PID.SetOutputRange(-1, 1);
//   ClimberAngle2PID.SetOutputRange(-1, 1);

//   //Sets inversion
//   ClimberAngle1.SetInverted(true);
//   ClimberAngle2.SetInverted(true);

//   //moves to motor controller
//   ClimberAngle1.BurnFlash();
//   ClimberAngle2.BurnFlash();

//   //Climber Angle PID
//   ClimberAngle1PID.SetP(0.6, 0);
//   ClimberAngle2PID.SetP(0.6, 0);

//   //Restores Factory defult
//   Climber1.RestoreFactoryDefaults();
//   Climber2.RestoreFactoryDefaults();

//   //Min and max Power
//   Climber1PID.SetOutputRange(0, .5);
//   Climber2PID.SetOutputRange(0, .5);

//   //Position movement power
//   Climber2.SetClosedLoopRampRate(.5);
//   Climber1.SetClosedLoopRampRate(.5);

//   //Sets the Inversions
//   Climber1.SetInverted(true);
//   Climber2.SetInverted(true);

//   //Moves to motor controller
//   Climber2.BurnFlash();
//   Climber1.BurnFlash();

  //Climber PID
  // Climber1PID.SetP(.041, 0);
  // Climber2PID.SetP(.041, 0);
  // Climber1PID.SetI(0, 0);//.0000005
  // Climber2PID.SetI(0, 0);
  // Climber1PID.SetD(0, 0);//.0006
  // Climber2PID.SetD(0, 0);

//}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  /*if(Hang == 0)
  {
    Climber1PID.SetReference(0, rev::ControlType::kPosition);
    Climber2PID.SetReference(0, rev::ControlType::kPosition);
  }*/
    //std::shared_ptr/*<NetworkTable>*/ table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    /*double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    double targetArea =  table->GetNumber("ta",0.0);
    double targetSkew = table->GetNumber("ts",0.0); 
    double targetValid = table->GetNumber("tv",0.0);
    double latency = table->GetNumber("tl",0.0);
    double shortSidelength = table->GetNumber("tshort",0.0);
    double longSidelength = table->GetNumber("tlong",0.0);
    double horizontalSidelength = table->GetNumber("thor",0.0);
    double verticalSidelength = table->GetNumber("tvert",0.0);
    double activePipeline = table->GetNumber("getpipe",0.0);
    double threeD_position = table->GetNumber("camtram",0.0);
    double xCorrdinates_arrays = table->GetNumber("tcornx",0.0);
    double yCorrdinates_arrays = table->GetNumber("tcorny",0.0);
    double xRaw_screenspace = table->GetNumber("tx0",0.0);
    double yRaw_screenspace = table->GetNumber("ty0",0.0);
    double area_percent = table->GetNumber("ta0",0.0);
    double skewRotation_degree = table->GetNumber("ts0",0.0);
    double screenspaceX_Raw = table->GetNumber("tx1",0.0);
    double screenspaceY_Raw = table->GetNumber("ty1",0.0);
    double image_Area = table->GetNumber("ta1",0.0);
    double Rotation_skew = table->GetNumber("ts1",0.0);    
    double Y_rawScreenspace = table->GetNumber("tx2",0.0);    
    double x_rawScreenspace = table->GetNumber("ty2",0.0);
    double Area_image = table->GetNumber("ta2",0.0);
    double Skew_rotation = table->GetNumber("ts2",0.0);
    double xCrossheir_A = table->GetNumber("cx0",0.0);
    double yCrossheir_A = table->GetNumber("cy0",0.0);
    double xCrossheir_B = table->GetNumber("cx1",0.0);
    double yCrossheir_B = table->GetNumber("cy1",0.0);
    frc::SmartDashboard::PutNumber("targetArea", targetArea);
    frc::SmartDashboard::PutNumber("targetSkew",targetSkew);
    frc::SmartDashboard::PutNumber("tx",targetOffsetAngle_Horizontal);
    frc::SmartDashboard::PutNumber("ty",targetOffsetAngle_Vertical);*/

}

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
void Robot::AutonomousInit() 
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kTwoBall) 
  {
    // Custom Auto goes here
  } 
  if(m_autoSelected == kFourBall)
  {

  }
  if(m_autoSelected == kFiveBall)
  {

  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() //Total of 12 to 16 trajectories
{ 
  frc::SmartDashboard::PutNumber("Case", A);
  Pigeon.GetFusedHeading(Status);
  m_odometry.Update(PigeonToRotation(Status.heading), units::length::inch_t(ClicksToInch(LeftDrive1.GetSelectedSensorPosition())),
  units::length::inch_t(ClicksToInch(RightDrive1.GetSelectedSensorPosition())));
  frc::Pose2d FieldPosition = m_odometry.GetPose();
  frc::SmartDashboard::PutNumber("PathTime", PathTime.Get().value());
  frc::SmartDashboard::PutNumber("Trajectory3time", trajectory3.TotalTime().value());
  frc::SmartDashboard::PutNumber("OdometryTrackerX", FieldPosition.X().value());
  frc::SmartDashboard::PutNumber("OdometryTrackerY", FieldPosition.Y().value());
  frc::SmartDashboard::PutNumber("OdometryTrackerAngle", FieldPosition.Rotation().Degrees().value());
  frc::SmartDashboard::PutNumber("ShooterAngle", ShooterAngle.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("ShooterIterm", ShooterAngle.GetIntegralAccumulator());

  if (m_autoSelected == kTwoBall)
  {
    // Custom Auto goes here
      switch (A)
      {
        //Shoot
        case 5:
        {
          ShooterAngle.Config_kP(0, 1.25); //.01
          ShooterAngle.Config_kI(0, .000033); //.000035
          ShooterAngle.Config_kD(0, .1);
          ShooterAngle.Set(ControlMode::Position, 80); //4500
          TopShooterPID.SetReference(Higher, rev::ControlType::kVelocity); //lower
          BottomShooterPID.SetReference(Higher+500, rev::ControlType::kVelocity); //lower
          if(ShooterAngle.GetSelectedSensorPosition()>=75)
          {
            A = 6;
          }
        }
        break;
        case 6:
        {
          Intaking.Reset();
          Intaking.Start();
          A = 8;
        }
        break;
        case 8:
        {
          if(BottomShooterEncoder.GetVelocity()>=Lower+400)
          {
            WheelStopper.Set(true);
            Magazine.Set(-.65);
            RightDrive1.Set(ControlMode::PercentOutput, 0);
            LeftDrive1.Set(ControlMode::PercentOutput, 0);
            frc::SmartDashboard::PutNumber("intakeTimer", Intaking.Get().value());
            if(Intaking.Get()>= units::second_t(2))
            {
              A = 10;
            }
          }
        }
        break;
        case 10:
        {
          //resets
          //stop motors
          TopShooter.Set(0);
          BottomShooter.Set(0);
          Magazine.Set(0);
          LeftDrive1.SetSelectedSensorPosition(0);
          RightDrive1.SetSelectedSensorPosition(0);
          m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
          PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
          A = 15; //20
          PathTime.Start();
        }
        break;
        case 15:
        {
          //Intake
          ShooterAngle.Config_kP(0, .15); //.007
          ShooterAngle.Config_kI(0, .00008); //..000005
          ShooterAngle.Config_kD(0, 0);
          IntakePosition.Set(frc::DoubleSolenoid::Value::kForward);
          Intake1.Set(ControlMode::PercentOutput, 1);
          ShooterAngle.Set(ControlMode::Position, -1100);
          TopShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
          BottomShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
          if(ShooterAngle.GetSelectedSensorPosition()<=-1100)
          {
            A = 20;
          }
        }
        break;
        case 20:
        {
          //move backwards
          const frc::Trajectory::State goal5 = trajectory5.Sample(PathTime.Get());
          frc::ChassisSpeeds adjustedSpeeds5 = PathFollower5.Calculate(FieldPosition, goal5);
          auto [left5, right5] = DriveKin.ToWheelSpeeds(adjustedSpeeds5);
          RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left5));
          LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right5));
          if(PathTime.HasElapsed(trajectory5.TotalTime()+units::time::second_t(.3)))
          {
            PathTime.Stop();
            // PathTime.Reset();
            // PathTime.Start();
            //A = 0; //30
          }
          if(Bottom1.GetValue()<=600 and Bottom2.GetValue() <=600)
          {
            //All wheels Move
            Magazine.Set(.65);
            WheelStopper.Set(true);
          }
          else if(Bottom1.GetValue()>=600 and Bottom2.GetValue()<=600)
          {
            //Back wheels stop
            WheelStopper.Set(false);
            Magazine.Set(.65);
            
          }
          else if(Bottom1.GetValue()>=600 and Bottom2.GetValue()>=600)
          {
            //All wheels stop
            Magazine.Set(0);
            WheelStopper.Set(false);
            A = 25;
          }
        }
        break;
        case 25:
        {
          //Intake in
          //ShooterAngle goes to zero
          //Everything else shuts off
          ShooterAngle.Set(ControlMode::Position, 0);
          RightDrive1.Set(ControlMode::PercentOutput, 0);
          LeftDrive1.Set(ControlMode::PercentOutput, 0);
          Magazine.Set(0);
          TopShooter.Set(0);
          BottomShooter.Set(0);
          IntakePosition.Set(frc::DoubleSolenoid::kReverse);
          Intake1.Set(ControlMode::PercentOutput, 0);
          if(ShooterAngle.GetSelectedSensorPosition()>=-5)
          {
            A = 27;
          }
          
        }
        break;
        case 27:
        {
          ShooterAngle.SetIntegralAccumulator(0,0,0);
          ShooterAngle.Config_kP(0, 1.35);
          ShooterAngle.Config_kI(0, 0);
          if(ShooterAngle.GetSelectedSensorPosition()>=-100)
          {
            A = 30;
          }
        }
        break;
        case 30:
        {
          ShooterAngle.Config_kP(0, 1.25); //.01
          ShooterAngle.Config_kI(0, 0); //.000033
          ShooterAngle.Config_kD(0, .1);
          ShooterAngle.Set(ControlMode::Position, 250);//80
          TopShooterPID.SetReference(3300, rev::ControlType::kVelocity);
          BottomShooterPID.SetReference(3500, rev::ControlType::kVelocity);
          if(ShooterAngle.GetSelectedSensorPosition()>=240 and BottomShooterEncoder.GetVelocity()>=3490)
          {
            WheelStopper.Set(false);
            Magazine.Set(-.65);
            if(BottomShooterEncoder.GetVelocity()>=3490 and ShooterAngle.GetSelectedSensorPosition()>=240 and Bottom1.GetValue()>600)
            {
              if(!TimerStart)
              {
                Shot.Start();
                TimerStart = true;
              }
              if(Shot.Get().value()>1)
              {
                WheelStopper.Set(true);
              }
            }
          }
        }
        break;
      }
    
  } 
//   if(m_autoSelected == kFourBall) //4-5 trajectories depending on where we shoot from
//   { 
//     switch (C) 
//     {
//       //Shoot one
//       case 10:
//       {
//         //resets
//         LeftDrive1.SetSelectedSensorPosition(0);
//         RightDrive1.SetSelectedSensorPosition(0);
//         m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
//         PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
//         C = 20;
//         PathTime.Start();
//       }
//       break;
//       //Intake
//       //Intake out
//       //Intake on
//       break;
//       case 20:
//       {
//         //move backwards
//         const frc::Trajectory::State goal7 = trajectory7.Sample(PathTime.Get());
//         frc::ChassisSpeeds adjustedSpeeds7 = PathFollower7.Calculate(FieldPosition, goal7);
//         auto [left7, right7] = DriveKin.ToWheelSpeeds(adjustedSpeeds7);
//         RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left7));
//         LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right7));
//         if(PathTime.HasElapsed(trajectory7.TotalTime()+units::time::second_t(.3)))
//         {
//           PathTime.Stop();
//           PathTime.Reset();
//           PathTime.Start();
//           C = 30;
//         }
//       break;
//       }
//       //Another trajectory to grab other ball
//       //Make sure intake is still on

//       case 30:
//       {
//         //moves back to starting position (current trajectory is subject to change since ending position will change)
//         const frc::Trajectory::State goal8 = trajectory8.Sample(PathTime.Get());
//         frc::ChassisSpeeds adjustedSpeeds8 = PathFollower8.Calculate(FieldPosition, goal8);
//         auto [left8, right8] = DriveKin.ToWheelSpeeds(adjustedSpeeds8);
//         RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left8));
//         LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right8));
//         if(PathTime.HasElapsed(trajectory8.TotalTime()+units::time::second_t(.3)))
//         {
//           PathTime.Stop();
//           PathTime.Reset();
//           PathTime.Start();
//           C = 40;
//         }
//       }
//       break;
//       //Shoot 2 in case 40
//       //zero everything case 50
//       //Turn intake on and out case 60
//       //Move to alliance station wall pick up 1 (trajectory) case 70
//       // move to shoot (trajectory) case 80
//       //shoot 1

    
    //}
//   }
//   if(m_autoSelected == kFiveBall) //5-6 trajectories depending on where we shoot from
//   {  
//     switch (D) 
//     {
//       //Shoot
//       case 10:
//       {
//         //resets
//         LeftDrive1.SetSelectedSensorPosition(0);
//         RightDrive1.SetSelectedSensorPosition(0);
//         m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
//         PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
//         D = 20;
//         PathTime.Start();
//       }
//       break;
//       //Intake
//       case 20:
//       {
//         //move backwards
//         const frc::Trajectory::State goal9 = trajectory9.Sample(PathTime.Get());
//         frc::ChassisSpeeds adjustedSpeeds9 = PathFollower9.Calculate(FieldPosition, goal9);
//         auto [left9, right9] = DriveKin.ToWheelSpeeds(adjustedSpeeds9);
//         RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left9));
//         LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right9));
//         if(PathTime.HasElapsed(trajectory9.TotalTime()+units::time::second_t(.3)))
//           {
//             PathTime.Stop();
//             PathTime.Reset();
//             PathTime.Start();
//             D = 30;
//           }
//       }
//       break;
//       case 30:
//         {
//           //moves back to starting position
//           const frc::Trajectory::State goal10 = trajectory10.Sample(PathTime.Get());
//           frc::ChassisSpeeds adjustedSpeeds10 = PathFollower10.Calculate(FieldPosition, goal10);
//           auto [left10, right10] = DriveKin.ToWheelSpeeds(adjustedSpeeds10);
//           RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left10));
//           LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right10));
//           if(PathTime.HasElapsed(trajectory10.TotalTime()+units::time::second_t(.3)))
//           {
//             PathTime.Stop();
//             PathTime.Reset();
//             PathTime.Start();
//             D = 40;
//           }
//         }
//         break;
//         //Shoot
//         //Same thing as four ball but shoot 2 at the end
//     }
//   }
  else //2-3 trajectories depending on where we shoot from
  {
    // Default Auto goes here (Three ball)
    switch (A)
    {
      //Shoot
        case 10:
        {
          //resets
          //stop motors
          TopShooter.Set(0);
          BottomShooter.Set(0);
          Magazine.Set(0);
          LeftDrive1.SetSelectedSensorPosition(0);
          RightDrive1.SetSelectedSensorPosition(0);
          m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
          PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
          A = 15; //20
          PathTime.Start();
        }
        break;
        case 15:
        {
          //Intake
          ShooterAngle.Config_kP(0, .15); //.007
          ShooterAngle.Config_kI(0, .00008); //..000005
          ShooterAngle.Config_kD(0, 0);
          IntakePosition.Set(frc::DoubleSolenoid::Value::kForward);
          Intake1.Set(ControlMode::PercentOutput, 1);
          ShooterAngle.Set(ControlMode::Position, -1100);
          TopShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
          BottomShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
          if(ShooterAngle.GetSelectedSensorPosition()<=-1100)
          {
            A = 20;
          }
        }
        break;
        case 20:
        {
          //move backwards
          const frc::Trajectory::State goal = trajectory.Sample(PathTime.Get());
          frc::ChassisSpeeds adjustedSpeeds = PathFollower.Calculate(FieldPosition, goal);
          auto [left, right] = DriveKin.ToWheelSpeeds(adjustedSpeeds);
          RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left));
          LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right));
          if(PathTime.HasElapsed(trajectory.TotalTime()+units::time::second_t(.3)))
          {
            PathTime.Stop();
            // PathTime.Reset();
            // PathTime.Start();
            //A = 0; //30
          }
          if(Bottom1.GetValue()<=600 and Bottom2.GetValue() <=600)
          {
            //All wheels Move
            Magazine.Set(.65);
            WheelStopper.Set(true);
          }
          else if(Bottom1.GetValue()>=600 and Bottom2.GetValue()<=600)
          {
            //Back wheels stop
            WheelStopper.Set(false);
            Magazine.Set(.65);
            
          }
          else if(Bottom1.GetValue()>=600 and Bottom2.GetValue()>=600)
          {
            //All wheels stop
            Magazine.Set(0);
            WheelStopper.Set(false);
            A = 25;
          }
        }
        break;
        case 25:
        {
          //Intake in
          //ShooterAngle goes to zero
          //Everything else shuts off
          ShooterAngle.Set(ControlMode::Position, 0);
          RightDrive1.Set(ControlMode::PercentOutput, 0);
          LeftDrive1.Set(ControlMode::PercentOutput, 0);
          Magazine.Set(0);
          TopShooter.Set(0);
          BottomShooter.Set(0);
          IntakePosition.Set(frc::DoubleSolenoid::kReverse);
          Intake1.Set(ControlMode::PercentOutput, 0);
          if(ShooterAngle.GetSelectedSensorPosition()>=-5)
          {
            A = 27;
          }
          
        }
        break;
        case 27:
        {
          ShooterAngle.SetIntegralAccumulator(0,0,0);
          ShooterAngle.Config_kP(0, 1.35);
          ShooterAngle.Config_kI(0, 0);
          if(ShooterAngle.GetSelectedSensorPosition()>=-100)
          {
            A = 30;
          }
        }
        break;
        case 30:
        {
          ShooterAngle.Config_kP(0, 1.15); //.01
          ShooterAngle.Config_kI(0, 0); //.000033
          ShooterAngle.Config_kD(0, .1);
          ShooterAngle.Set(ControlMode::Position, 250);//80
          TopShooterPID.SetReference(3300, rev::ControlType::kVelocity);
          BottomShooterPID.SetReference(3500, rev::ControlType::kVelocity);
          if(ShooterAngle.GetSelectedSensorPosition()>=240 and BottomShooterEncoder.GetVelocity()>=3490)
          {
            
            WheelStopper.Set(false);
            Magazine.Set(-.65);
            if(BottomShooterEncoder.GetVelocity()>=3490 and ShooterAngle.GetSelectedSensorPosition()>=240 and Bottom1.GetValue()>600)
            {
              if(!TimerStart)
              {
                Shot.Start();
                TimerStart = true;
              }
              if(Shot.Get().value()>1)
              {
                WheelStopper.Set(true);
              }
            }
          }
        }
        break;
      }
  }
}
void Robot::TeleopInit() 
{
  //reset pigeon value to zero
  Pigeon.SetFusedHeading(0, 30);
}

void Robot::TeleopPeriodic() 
{
  frc::SmartDashboard::PutNumber("ShooterPosition", ShooterAngle.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("OldEncoderValue", ShooterAngle.GetSelectedSensorPosition(1));
  frc::SmartDashboard::PutNumber("ShooterAnglePercentOut", ShooterAngle.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("ShooterClosedLoopError", ShooterAngle.GetClosedLoopError());
  frc::SmartDashboard::PutNumber("ClimbAngle1", ClimberAngle1Encoder.GetPosition());
  frc::SmartDashboard::PutNumber("ClimberAngle2", ClimberAngle2Encoder.GetPosition());
  frc::SmartDashboard::PutNumber("ClimberHeight1", Climber1Encoder.GetPosition());
  frc::SmartDashboard::PutNumber("ClimberHeight2", Climber2Encoder.GetPosition());
  frc::SmartDashboard::PutNumber("TopShootSpeed", TopShooterEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("BottomShooterSpeed", BottomShooterEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("ShooterSpeed", ShooterAngle.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("HangingCase", Hang);
  frc::SmartDashboard::PutNumber("ITermShooter", ShooterAngle.GetIntegralAccumulator());
  //frc::SmartDashboard::PutNumber("BackSwitch", BackSwitch.Get());
  //frc::SmartDashboard::PutNumber("SideSwitch", SideSwitch.Get());
  frc::SmartDashboard::PutNumber("Bottom1Value", Bottom1.GetValue());
  frc::SmartDashboard::PutNumber("Bottom2Value", Bottom2.GetValue());
  frc::SmartDashboard::PutNumber("ClimberAnglePValue", ClimberAngle1PID.GetP());
  frc::SmartDashboard::PutNumber("ShooterCode", Manipulator.GetStartButton());
  frc::SmartDashboard::PutNumber("Test", Manipulator.GetBackButtonPressed());
  frc::SmartDashboard::PutNumber("ShooterCodeValue", ShooterCode);

  //Drive off of Joysticks
  RightDrive1.Set(ControlMode::PercentOutput, Driver.GetRightY()*-.55);
  LeftDrive1.Set(ControlMode::PercentOutput, Driver.GetLeftY()*-.55);
    
  /*if(Driver.GetXButton()==1)
  {
    ClimberAngle1.SetSelectedSensorPosition(0, 0);
    ClimberAngle2.SetSelectedSensorPosition(0, 0);
  }
  else
  {
    ClimberAngle1.Set(ControlMode::PercentOutput, 0);
    ClimberAngle2.Set(ControlMode::PercentOutput, 0);
  }*/
  /*if(Driver.GetYButton()==1)
  {
    ClimberAngle1.Set(ControlMode::Position, 6000);
    ClimberAngle2.Set(ControlMode::Position, 6000);
  }
  else
  {
    ClimberAngle1.Set(ControlMode::PercentOutput, 0);
    ClimberAngle2.Set(ControlMode::PercentOutput, 0);
  }*/
  /*if(Driver.GetXButton()==1 and Driver.GetYButton()==0)
  {
    ShooterAngle.SetSelectedSensorPosition(0);
  }
  else if(Driver.GetXButton()==0 and Driver.GetYButton()==1)
  {
    ShooterAngle.Set(ControlMode::Position, 6000);
  }
  else 
  {
    ShooterAngle.Set(ControlMode::PercentOutput, 0);
  }*/
  
  if(Driver.GetYButtonPressed()==1)
  {
    ShooterCode=!ShooterCode;
  }
  //Climbing Shooter position
  if(ShooterCode == true)
  {
    if(Manipulator.GetLeftTriggerAxis()==1)
    {
      ShooterAngle.Config_kP(0, .15); //.007
      ShooterAngle.Config_kI(0, .00008); //..000005
      ShooterAngle.Config_kD(0, 0);
      IntakePosition.Set(frc::DoubleSolenoid::Value::kForward);
      if(IntakePosition.GetFwdChannel()==1)
      {
        ShooterAngle.Set(ControlMode::PercentOutput, Manipulator.GetRightY()*.25);
      }
    }
    else
    {

      ShooterAngle.Config_kP(0, 0); 
      ShooterAngle.Config_kI(0, 0); 
      ShooterAngle.Config_kD(0, 0);
      //ShooterAngle.Set(ControlMode::PercentOutput, 0);
      IntakePosition.Set(frc::DoubleSolenoid::Value::kReverse);
    }
  }
  //Normal Teleop Shooting
  else
  {
  //Intaking
  if(Driver.GetRightTriggerAxis()==0 and Driver.GetLeftTriggerAxis()==1 
  and Manipulator.GetAButton()==0 and Manipulator.GetBButton()==0 
  and Manipulator.GetYButton()==0 and Manipulator.GetXButton()==0)
  {
    ShooterAngle.Config_kP(0, .15); //.007
    ShooterAngle.Config_kI(0, .00008); //..000005
    ShooterAngle.Config_kD(0, 0);
    IntakePosition.Set(frc::DoubleSolenoid::Value::kForward);
    TopShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
    BottomShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
    if(IntakePosition.GetFwdChannel()==1)
    {
      ShooterAngle.Set(ControlMode::Position, -1100);
    }
    if(ShooterAngle.GetSelectedSensorPosition()<=-1000)
    {
      Intake1.Set(ControlMode::PercentOutput, 1);
    }
  }
  
  //reverse intake wheels
  else if(Driver.GetRightTriggerAxis()==1 and Driver.GetLeftTriggerAxis()==0 
  and Manipulator.GetAButton()==0 and Manipulator.GetBButton()==0 
  and Manipulator.GetYButton()==0 and Manipulator.GetXButton()==0)
  {
    IntakePosition.Set(frc::DoubleSolenoid::Value::kForward);
    Intake1.Set(ControlMode::PercentOutput, -.5);
  }

  //Forward shot high
  else if (Driver.GetRightTriggerAxis()==0 and Driver.GetLeftTriggerAxis()==0 
  and Manipulator.GetAButton()==1 and Manipulator.GetBButton()==0 
  and Manipulator.GetYButton()==0 and Manipulator.GetXButton()==0)
  {
    ShooterAngle.Config_kP(0, 1.25); //.01
    ShooterAngle.Config_kI(0, .000033); //.000035
    ShooterAngle.Config_kD(0, .1);
    TopShooterPID.SetReference(Higher, rev::ControlType::kVelocity);
    BottomShooterPID.SetReference(Higher+500, rev::ControlType::kVelocity);
    ShooterAngle.Set(ControlMode::Position, -110); //-4500
    if(BottomShooterEncoder.GetVelocity()>=Higher+400 and ShooterAngle.GetSelectedSensorPosition()<=-105 and ShooterAngle.GetSelectedSensorPosition()>=-130) //4500
    {
      WheelStopper.Set(false);
      Magazine.Set(-.45);
      if(BottomShooterEncoder.GetVelocity()>=Higher+400 and ShooterAngle.GetSelectedSensorPosition()<=-105 and Bottom1.GetValue()>600 and
      ShooterAngle.GetSelectedSensorPosition()>=-130) //-4500
      {
        WheelStopper.Set(true);
      }
    }
  }
  
  //Backward shot low
  else if (Driver.GetRightTriggerAxis()==0 and Driver.GetLeftTriggerAxis()==0 
  and Manipulator.GetAButton()==0 and Manipulator.GetBButton()==1 
  and Manipulator.GetYButton()==0 and Manipulator.GetXButton()==0)
  {
    ShooterAngle.Config_kP(0, 1.25); //.01
    ShooterAngle.Config_kI(0, .000033); //.000035
    ShooterAngle.Config_kD(0, .1);
    TopShooterPID.SetReference(Lower, rev::ControlType::kVelocity);
    BottomShooterPID.SetReference(Lower+500, rev::ControlType::kVelocity);
    ShooterAngle.Set(ControlMode::Position, -220);
    if(BottomShooterEncoder.GetVelocity()>=Lower+400 and ShooterAngle.GetSelectedSensorPosition()>=-250 and ShooterAngle.GetSelectedSensorPosition()<=-300)
    {
      WheelStopper.Set(false);
      Magazine.Set(-.45);
      if(TopShooterEncoder.GetVelocity()>=Lower-100 and ShooterAngle.GetSelectedSensorPosition()>=-250 and Bottom1.GetValue()>600 and ShooterAngle.GetSelectedSensorPosition()<=-300)
      {
        WheelStopper.Set(true);
      }
    }
  }

  //Safe Shot
  else if (Driver.GetRightTriggerAxis()==0 and Driver.GetLeftTriggerAxis()==0 
  and Manipulator.GetAButton()==0 and Manipulator.GetBButton()==0 
  and Manipulator.GetYButton()==1 and Manipulator.GetXButton()==0)
  {
    ShooterAngle.Config_kP(0, 1.05); //.01
    ShooterAngle.Config_kI(0, .000033); //.000035
    ShooterAngle.Config_kD(0, .1);
    TopShooterPID.SetReference(Safe, rev::ControlType::kVelocity);
    BottomShooterPID.SetReference(Safe+200, rev::ControlType::kVelocity);
    ShooterAngle.Set(ControlMode::Position, -260);
    if(BottomShooterEncoder.GetVelocity()>=Safe+190 and ShooterAngle.GetSelectedSensorPosition()<=-255 
    and ShooterAngle.GetSelectedSensorPosition()>=-265)
    {
      WheelStopper.Set(false);
      Magazine.Set(-.65);
      if(BottomShooterEncoder.GetVelocity()>=Safe+190 and ShooterAngle.GetSelectedSensorPosition()<=-255 
      and Bottom1.GetValue()>600 and ShooterAngle.GetSelectedSensorPosition()>=-265)
      {
        WheelStopper.Set(true);
      }
    }
  }
  
  //Back Shot High
  else if (Driver.GetRightTriggerAxis()==0 and Driver.GetLeftTriggerAxis()==0 
  and Manipulator.GetAButton()==0 and Manipulator.GetBButton()==0 
  and Manipulator.GetYButton()==0 and Manipulator.GetXButton()==1)
  {
    ShooterAngle.Config_kP(0, 1.25); //.01
    ShooterAngle.Config_kI(0, .000033); //.000035
    ShooterAngle.Config_kD(0, .1);
    TopShooterPID.SetReference(Higher, rev::ControlType::kVelocity);
    BottomShooterPID.SetReference(Higher+500, rev::ControlType::kVelocity);
    ShooterAngle.Set(ControlMode::Position, 80); //-4500
    if(BottomShooterEncoder.GetVelocity()>=Higher+400 and ShooterAngle.GetSelectedSensorPosition()>=80 and ShooterAngle.GetSelectedSensorPosition()<=130) 
    {
      WheelStopper.Set(false);
      Magazine.Set(-.45);
      if(BottomShooterEncoder.GetVelocity()>=Higher+400 and ShooterAngle.GetSelectedSensorPosition()>=80 and Bottom1.GetValue()>600
      and ShooterAngle.GetSelectedSensorPosition()<=130) 
      {
        WheelStopper.Set(true);
      }
    }
  }
  
  //everything turns off
  else
  {
    ShooterAngle.Config_kP(0, .4); //.01
    ShooterAngle.Config_kI(0, .000033); //.000035
    ShooterAngle.Config_kD(0, .1);
    ShooterAngle.Set(ControlMode::Position, 0);
    //ShooterAngle.Set(ControlMode::PercentOutput, 0);
    Intake1.Set(ControlMode::PercentOutput, 0);
    TopShooter.Set(0);
    BottomShooter.Set(0);
    Magazine.Set(0);
    if(ShooterAngle.GetSelectedSensorPosition()>=-1100)
    {
      IntakePosition.Set(frc::DoubleSolenoid::Value::kReverse);
    }
  }

    //All wheels move
    // if(Driver.GetLeftTriggerAxis()==1 and BackSwitch.Get() == 1 and SideSwitch.Get() == 1)
    //   {
    //     Magazine.Set(.65);
    //     WheelStopper.Set(true);
    //   }
    // //Back wheels stop
    // else if(Driver.GetLeftTriggerAxis()==1 and BackSwitch.Get() == 0 and SideSwitch.Get() == 1)
    // {
    //   WheelStopper.Set(false);
    //   Magazine.Set(.65);
    // }
    // //All wheels stop
    // else if(Driver.GetLeftTriggerAxis()==1 and BackSwitch.Get() == 0 and SideSwitch.Get() == 0)
    // {
    //   Magazine.Set(0);
    //   WheelStopper.Set(false);
    // }
    if(Driver.GetLeftTriggerAxis()==1 and Bottom1.GetValue()<=600 and Bottom2.GetValue()<=600)
      {
        //All wheels Move
        Magazine.Set(.65);
        WheelStopper.Set(true);
      }
    else if(Driver.GetLeftTriggerAxis()==1 and Bottom1.GetValue()>=600 and Bottom2.GetValue()<=600)
    {
      //Back wheels stop
      Magazine.Set(.65);
      WheelStopper.Set(false);
    }
    else if(Driver.GetLeftTriggerAxis()==1 and Bottom1.GetValue()>=600 and Bottom2.GetValue()>=600)
    {
      //All wheels stop
      Magazine.Set(0);
      WheelStopper.Set(false);
    }

    //resets integral to zero every time button is hit
    if(Manipulator.GetAButtonPressed()==1 or Manipulator.GetAButtonReleased()==1 
    or Manipulator.GetBButtonPressed()==1 or Manipulator.GetBButtonReleased()==1 
    or Manipulator.GetXButtonPressed()==1 or Manipulator.GetXButtonReleased()==1 
    or(Driver.GetLeftTriggerAxis()<.7 and Driver.GetLeftTriggerAxis()>.2))
    {
      ShooterAngle.SetIntegralAccumulator(0,0,0);
    }
  }

  //  Climber1.Set(Manipulator.GetLeftY()*.1);
  //  Climber2.Set(Manipulator.GetLeftY()*.1);
  // if(Driver.GetXButton()==1 and Manipulator.GetLeftBumperPressed()==0)
  // {
  //   Climber1PID.SetP(.041, 0);
  //   Climber2PID.SetP(.041, 0);

  //   Climber1PID.SetI(0, 0);//.0000005
  //   Climber2PID.SetI(0, 0);

  //   Climber1PID.SetD(0, 0);//.0006
  //   Climber2PID.SetD(0, 0);

  //   Climber1PID.SetReference(0, rev::ControlType::kPosition);
  //   Climber2PID.SetReference(0, rev::ControlType::kPosition);
  //   // Climber1Encoder.SetPosition(0);
  //   // Climber2Encoder.SetPosition(0);
  // }
  //Positive = down Rachet = false
  //negative = up Rachet = true
  // if(Manipulator.GetRightBumper()==0 and Manipulator.GetLeftBumper()==1)
  // {
  //   ClimberAngle1.Set(.1*Manipulator.GetRightY());
  //   ClimberAngle2.Set(.1*Manipulator.GetRightY());
  // }
  // else if(Manipulator.GetRightBumper()==1 and Manipulator.GetLeftBumper()==0)
  // {
  //   Climber1.Set(.05*Manipulator.GetLeftY());
  //   Climber2.Set(.05*Manipulator.GetLeftY());
  // }
  // else
  // {
  //   Climber1.Set(0);
  //   Climber2.Set(0);
  //   ClimberAngle1.Set(0);
  //   ClimberAngle2.Set(0);
  // }


  if(Manipulator.GetLeftBumperPressed())
  {
    ClimbRachet.Set(true);

    Climber1PID.SetP(.045, 0);
    Climber2PID.SetP(.045, 0);
    Climber1PID.SetI(0, 0);//.0000005
    Climber2PID.SetI(0, 0);
    Climber1PID.SetD(0, 0);//.0006
    Climber2PID.SetD(0, 0);

    Hang = 10;
  }
    switch (Hang)
      {
        case 0:
        {
          if(Manipulator.GetRightBumper()==1)
          {
            ClimbRachet.Set(false);
            Climber1.Set(.5*Manipulator.GetLeftY());
            Climber2.Set(.5*Manipulator.GetLeftY());
            // ClimberAngle1.Set(.1*Manipulator.GetRightY());
            // ClimberAngle2.Set(.1*Manipulator.GetRightY());
          }
            
          // Climber1PID.SetReference(2, rev::ControlType::kPosition);
          // Climber2PID.SetReference(2, rev::ControlType::kPosition);
          ClimberAngle1PID.SetReference(0, rev::ControlType::kPosition);
          ClimberAngle2PID.SetReference(0, rev::ControlType::kPosition);
        }
        break;
        case 10:
        {
          //Climb height arms extend and climb angle moves back
          //ClimbRachet.Set(false);
          ClimberAngle1PID.SetReference(-50.4, rev::ControlType::kPosition); 
          ClimberAngle2PID.SetReference(-50.4, rev::ControlType::kPosition); 
          Climber1PID.SetReference(-37, rev::ControlType::kPosition); 
          Climber2PID.SetReference(-37, rev::ControlType::kPosition); 
          if(ClimbRachet.Get()==true and Climber1Encoder.GetPosition()<=-1 and Climber2Encoder.GetPosition()<=-1)
          {
            //Changes speed climber height arms
            Climber1PID.SetOutputRange(-.5, .75);
            Climber2PID.SetOutputRange(-.5, .75);
            Climber1.BurnFlash();
            Climber2.BurnFlash();
            
          }

          if(Driver.GetAButtonPressed()==1 and Climber1Encoder.GetPosition()<-36.5 and ClimberAngle1Encoder.GetPosition()<-49 and ClimberAngle2Encoder.GetPosition()<-49)//Climber1Encoder.GetPosition()<=-49 and ClimberAngle1.GetSelectedSensorPosition()<=-9 and
          {
            
            Hang = 25; //25 0
            ClimbRachet.Set(false);
          }
          
        }
        break;
        case 25:
        {
          //Climber height arms retract/pull down
          //ClimbRachet.Set(false);
          Climber1PID.SetReference(-15, rev::ControlType::kPosition);//1
          Climber2PID.SetReference(-15, rev::ControlType::kPosition);//1
          if(Driver.GetAButtonPressed()==1 and Climber1Encoder.GetPosition()>-16)//Climber1Encoder.GetPosition()>=-1
          {
            //ClimbRachet.Set(false);
            Hang = 20;
          }
        }
        break;
        case 20:
        {
          //climb angle arms move in
          ClimberAngle1PID.SetReference(0, rev::ControlType::kPosition);
          ClimberAngle2PID.SetReference(0, rev::ControlType::kPosition);
          if(Driver.GetAButtonPressed()==1 and ClimberAngle1Encoder.GetPosition()>-.5 and ClimberAngle2Encoder.GetPosition()>-.5) //ClimberAngle1.GetSelectedSensorPosition()>=-10 
          {
            Climber1PID.SetOutputRange(-.05, .05);
            Climber2PID.SetOutputRange(-.05, .05);
            Climber1.BurnFlash();
            Climber2.BurnFlash();
            Hang = 27;
          }
        }
        break;
        case 27:
        {
          //Climb height arms extend up a little
          Climber1PID.SetReference(-5, rev::ControlType::kPosition);
          Climber2PID.SetReference(-5, rev::ControlType::kPosition);
          if(Driver.GetAButtonPressed()==1 and Climber1Encoder.GetPosition()<-4) //Climber1Encoder.GetPosition()<=-.5 and 
          {
            Hang = 29;
          }
        }
        break;
        case 29:
        {
          //Climb angle arms move forward 
          ClimberAngle1PID.SetReference(250, rev::ControlType::kPosition);
          ClimberAngle2PID.SetReference(250, rev::ControlType::kPosition);
          if(Driver.GetAButtonPressed()==1 and ClimberAngle1Encoder.GetPosition()>=249 and ClimberAngle2Encoder.GetPosition()>=249)
          {
            Climber1PID.SetOutputRange(-.5, .5);
            Climber2PID.SetOutputRange(-.5, .5);
            Climber1.BurnFlash();
            Climber2.BurnFlash();
            Hang = 30;
          }
        }
        break;
        case 30:
        {
          // Climber.Reset();
          // Climber.Start();
          //Climb height arms extend the full distance
          Climber1PID.SetReference(-30, rev::ControlType::kPosition);
          Climber2PID.SetReference(-30, rev::ControlType::kPosition);
          if(Driver.GetAButtonPressed()==1 and Climber1Encoder.GetPosition()<=-29) //Climber.Get()>= units::second_t(3)
          { 
            //Climber.Stop();
            Hang = 40;
          }
        }
        break;
        case 40:
        {
          //climb angle arms move back a little
          ClimberAngle1PID.SetReference(240, rev::ControlType::kPosition); //-63.3
          ClimberAngle2PID.SetReference(240, rev::ControlType::kPosition); //63.3
          if(Driver.GetAButtonPressed()==1 and ClimberAngle1Encoder.GetPosition()<=241 and ClimberAngle2Encoder.GetPosition()<=241) //Climber1Encoder.GetPosition()<=-49 and 
          {
            Hang = 45;//50
          }
        }
        break;
        case 45:
        {
          //Climb angle arms move back past climbing bar
          //Climb height arms pull down a little

          ClimberAngle1PID.SetReference(-50.4, rev::ControlType::kPosition);
          ClimberAngle2PID.SetReference(-50.4, rev::ControlType::kPosition);
          Climber1PID.SetReference(-20, rev::ControlType::kPosition);
          Climber2PID.SetReference(-20, rev::ControlType::kPosition);
          if(Driver.GetAButtonPressed()==1 and Climber1Encoder.GetPosition()>-21 and ClimberAngle1Encoder.GetPosition()<-50 and ClimberAngle2Encoder.GetPosition()<-50)
          {
            Hang = 50;
          }
        }
        break;
        case 50:
        {
          //Climb height arms pull down the rest of the way
          Climber1PID.SetReference(1, rev::ControlType::kPosition);
          Climber2PID.SetReference(1, rev::ControlType::kPosition);
          if(Driver.GetAButtonPressed()==1 and Climber1Encoder.GetPosition()<=1)
          {
            ClimbRachet.Set(false);
            Hang = 0;
          }
          
        }
        break;
        /*case 60:
        {
          if(!Traversal)
          {
            Traversal = true;
            Hang = 20;
          }
          else
          {
            Hang = 0;
          }
        }
        break;*/
        /*case 60:
        {
          Climber.Reset();
          Climber.Start();
          ClimberAngle1Encoder.SetPosition(0);
          ClimberAngle2Encoder.SetPosition(0);

          if(Climber.Get()>= units::second_t(3))
          {
            Climber.Stop();
            Hang = 70;
          }
        }
        break;
        case 70:
        {
          Climber1Encoder.SetPosition(2);
          Climber2Encoder.SetPosition(2);
          Hang = 80;
        }
        break;
        case 80:
        {
          Climber1Encoder.SetPosition(-2);
          Climber2Encoder.SetPosition(-2);
          Hang = 90;
        }
        break;*/
      }
  //}

  /*frc::Pose2d FieldPosition = m_odometry.GetPose();
  //Puts things on the Smart Dashboard
  frc::SmartDashboard::PutNumber("LeftPosition", LeftDrive1.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("RightPosition", RightDrive1.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Switch", TestSwitch.Get());
  frc::SmartDashboard::PutNumber("RightDriveInches", ClicksToInch(RightDrive1.GetSelectedSensorPosition()));
  frc::SmartDashboard::PutNumber("OdometryTrackerX", FieldPosition.X().value());
  frc::SmartDashboard::PutNumber("OdometryTrackerY", FieldPosition.Y().value());
  frc::SmartDashboard::PutNumber("OdometryTrackerAngle", FieldPosition.Rotation().Degrees().value());
  frc::SmartDashboard::PutBoolean("StartMath", StartMath);
  frc::SmartDashboard::PutNumber("PathTime", PathTime.Get().value());
  frc::SmartDashboard::PutNumber("LeftVelocity", LeftDrive1.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("RightVelocity", RightDrive1.GetSelectedSensorVelocity());
  //Updates the pigeon status
  Pigeon.GetFusedHeading(Status);
  //Puts pigeon on the smart dashboard
  frc::SmartDashboard::PutNumber("PidgeonValue", Status.heading);
    
  //Updates robot position on the field
  m_odometry.Update(PigeonToRotation(Status.heading), 
  units::length::inch_t(ClicksToInch(LeftDrive1.GetSelectedSensorPosition())),
  units::length::inch_t(ClicksToInch(RightDrive1.GetSelectedSensorPosition())));
  //Make the robot move according to planned trajectory
  if(Driver.GetBButtonPressed() == 1)
  {
    StartMath = !StartMath;
    PathTime.Start();
  }
  if(StartMath == true)
  {
    const frc::Trajectory::State goal2 = trajectory.Sample(PathTime.Get());//3.4_s
    frc::ChassisSpeeds adjustedSpeeds2 = PathFollower3.Calculate(FieldPosition, goal2);
    auto [left2, right2] = DriveKin.ToWheelSpeeds(adjustedSpeeds2);
    RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left2));
    LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right2));
    frc::SmartDashboard::PutNumber("LeftTargetVelocityMSec", left2.value());
    frc::SmartDashboard::PutNumber("RightTargetVelocityMSec", right2.value());
    frc::SmartDashboard::PutNumber("NumSetPoints", trajectory.TotalTime().value());
      
    frc::SmartDashboard::PutNumber("LeftTargetVelocityMSec", left3.value());
    frc::SmartDashboard::PutNumber("RightTargetVelocityMSec", right3.value());
    frc::SmartDashboard::PutNumber("NumSetPoints", trajectory4.TotalTime().value());
  }*/

  //Running Drive on limit switch
  /*if(TestSwitch.Get() == 1)
  {
    LeftDrive1.Set(ControlMode::Position, 10000);
    RightDrive1.Set(ControlMode::Position, 10000);
  } 
  else 
  {
    LeftDrive1.Set(ControlMode::PercentOutput, 0);
    RightDrive1.Set(ControlMode::PercentOutput, 0);
  }*/

  //Resets the drive encoder, resets field position, and updates pigeon values
  /*if(Driver.GetYButtonPressed()==1)
  {
    Pigeon.GetFusedHeading(Status);
    LeftDrive1.SetSelectedSensorPosition(0);
    RightDrive1.SetSelectedSensorPosition(0);
    m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
    PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
  }*/

  //timer on button
  /*if(Driver.GetAButton()==1)
  {
    Practice.Reset();
    Practice.Start();
    BottomShooterPID.SetReference(SetPoint, rev::ControlType::kVelocity);
    TopShooterPID.SetReference(SetPoint, rev::ControlType::kVelocity);
  }
  if(Practice.Get()>= units::second_t(5)) //or use time_to
  {
    Practice.Stop();
    TopShooter.Set(0);
    BottomShooter.Set(0);
  }*/
}


void Robot::DisabledInit() 
{}
void Robot::DisabledPeriodic()
{}
void Robot::TestInit()
{}
void Robot::TestPeriodic()
{}
 frc::Trajectory Robot::GenerateTrajectory()
{
  const frc::Pose2d sideStart{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{5.5_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{2_ft, 0_ft},
    frc::Translation2d{3_ft, 0_ft}};
  frc::TrajectoryConfig config{6_fps, 4_fps_sq};
  config.SetReversed(false);

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory;
}

frc::Trajectory Robot::GenerateTrajectory2()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory2;
}

frc::Trajectory Robot::GenerateTrajectory3()
{
  const frc::Pose2d sideStart{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 11_ft, frc::Rotation2d(180_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{3_ft, 5_ft}};
  frc::TrajectoryConfig config{12_fps, 2_fps_sq};
  config.SetReversed(false);

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory3;
}

frc::Trajectory Robot::GenerateTrajectory4()
{
  const frc::Pose2d sideStart{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{8_ft, 7_ft, frc::Rotation2d(90_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {};
  frc::TrajectoryConfig config{12_fps, 2_fps_sq};
  config.SetReversed(false);

  frc::Trajectory trajectory4 = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory4;
}

frc::Trajectory Robot::GenerateTrajectory5()
{
  const frc::Pose2d sideStart{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{8_ft, 0_ft, frc::Rotation2d(0_deg)}; //6
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{4_ft, 0_ft},
    frc::Translation2d{5_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 8_fps_sq};
  config.SetReversed(false);

  frc::Trajectory trajectory5 = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory5;
}

frc::Trajectory Robot::GenerateTrajectory6()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory6;
}

frc::Trajectory Robot::GenerateTrajectory7()
{
  const frc::Pose2d sideStart{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{4_ft, 0_ft},
    frc::Translation2d{5_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 8_fps_sq};
  config.SetReversed(false);

  frc::Trajectory trajectory5 = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory7;
}

frc::Trajectory Robot::GenerateTrajectory8()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory8;
}

frc::Trajectory Robot::GenerateTrajectory9()
{
  const frc::Pose2d sideStart{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{4_ft, 0_ft},
    frc::Translation2d{5_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 8_fps_sq};
  config.SetReversed(false);

  frc::Trajectory trajectory5 = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory9;
}

frc::Trajectory Robot::GenerateTrajectory10()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);

  return trajectory10;
}

/*frc::Trajectory Robot::GenerateTrajectory11()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);
  return trajectory11;
}
frc::Trajectory Robot::GenerateTrajectory12()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);
  return trajectory12;
}
frc::Trajectory Robot::GenerateTrajectory13()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);
  return trajectory13;
}
frc::Trajectory Robot::GenerateTrajectory14()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);
  return trajectory14;
}
frc::Trajectory Robot::GenerateTrajectory15()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);
  return trajectory15;
}
frc::Trajectory Robot::GenerateTrajectory16()
{
  const frc::Pose2d sideStart{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  const frc::Pose2d crossScale{0_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{5_ft, 0_ft},
    frc::Translation2d{4_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 5_fps_sq};
  config.SetReversed(true);
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory
  (sideStart, interiorWaypoints, crossScale, config);
  frc::SmartDashboard::PutBoolean("TrajectoryGeneration", true);
  return trajectory16;
}*/

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif
