// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

void Robot::RobotInit() 
{
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  m_chooser.SetDefaultOption(kThreeBall, kThreeBall);
  m_chooser.AddOption(kTwoBall, kTwoBall);
  m_chooser.AddOption(kFourBall, kFourBall);
  m_chooser.AddOption(kFiveBall, kFiveBall);
  
  //Compressor1.EnableDigital();
  Compressor1.Enabled();
  //Compressor1.Disable();
  //Compressor1.GetPressureSwitchValue();
  //Compressor1.GetCompressorCurrent();

  //Follow command
  RightDrive2.Follow(RightDrive1);
  LeftDrive2.Follow(LeftDrive1);

  //Controls Ramp Rate
  RightDrive1.ConfigClosedloopRamp(1);
  LeftDrive1.ConfigClosedloopRamp(1);

  ShooterAngle.ConfigClosedloopRamp(2.5);
  ShooterAngle.ConfigPeakOutputReverse(.1);

  //Power Limit
  RightDrive1.ConfigPeakOutputForward(.65);
  LeftDrive1.ConfigPeakOutputForward(.65);

  ShooterAngle.ConfigPeakOutputForward(.2);
  ShooterAngle.SetNeutralMode(Brake);

  ClimberAngle1.ConfigPeakOutputForward(.4);
  ClimberAngle2.ConfigPeakOutputForward(.4);

  Climber1PID.SetOutputRange(-.5, .5);
  Climber2PID.SetOutputRange(-.5, .5);

  TopShooter.SetClosedLoopRampRate(1.5);
  BottomShooter.SetClosedLoopRampRate(1.5);

  Climber2.BurnFlash();
  Climber1.BurnFlash();

  
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

  ClimberAngle1.ConfigFactoryDefault();
  ClimberAngle2.ConfigFactoryDefault();

  Climber1.RestoreFactoryDefaults();
  Climber2.RestoreFactoryDefaults();

  TopShooter.RestoreFactoryDefaults();
  BottomShooter.RestoreFactoryDefaults();

  ShooterAngle.ConfigFactoryDefault();

  Climber2.SetClosedLoopRampRate(.5);
  Climber1.SetClosedLoopRampRate(.5);

  //Setting Inversion
  RightDrive1.SetInverted(false);
  LeftDrive1.SetInverted(true);
  RightDrive2.SetInverted(InvertType::FollowMaster);
  LeftDrive2.SetInverted(InvertType::FollowMaster);

  ClimberAngle1.SetInverted(true);
  ClimberAngle2.SetInverted(true);

  Climber1.SetInverted(true);
  Climber2.SetInverted(false);

  Intake1.SetInverted(false);

  TopShooter.SetInverted(false);
  BottomShooter.SetInverted(true);
  Magazine.SetInverted(true);

  //Defines Drive Sensor
  LeftDrive1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  RightDrive1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

  //Defining Climb encoders
  ClimberAngle1.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
  ClimberAngle2.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

  //ShooterAngle.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Signed_PlusMinus180, 10);
  ShooterAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  

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

  ShooterAngle.Config_kP(0, .007); 
  ShooterAngle.Config_kI(0, .0000035); 
  ShooterAngle.Config_kD(0, 0); 
  ShooterAngle.Config_kF(0, 0);
  ShooterAngle.Config_IntegralZone(0, 0);

  Intake1.Config_kP(0, .1);

  Climber1PID.SetP(.08, 0);
  Climber2PID.SetP(.08, 0);

  Climber1PID.SetI(0, 0);//.0000005
  Climber2PID.SetI(0, 0);

  Climber1PID.SetD(0, 0);//.0006
  Climber2PID.SetD(0, 0);

  ClimberAngle1.Config_kP(0, .3);
  ClimberAngle2.Config_kP(0, .3);


  
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
  ShooterAngle.SetSelectedSensorPosition(0);
  frc::Pose2d FieldPosition = m_odometry.GetPose();
  frc::SmartDashboard::PutNumber("PathTime", PathTime.Get().value());
  frc::SmartDashboard::PutNumber("Trajectory3time", trajectory3.TotalTime().value());
  frc::SmartDashboard::PutNumber("OdometryTrackerX", FieldPosition.X().value());
  frc::SmartDashboard::PutNumber("OdometryTrackerY", FieldPosition.Y().value());
  frc::SmartDashboard::PutNumber("OdometryTrackerAngle", FieldPosition.Rotation().Degrees().value());

  if (m_autoSelected == kTwoBall) //Could change it to be 1 trajectory depending on where we shoot from
  {
    // Custom Auto goes here
      switch (A)
      {
        //Shoot
        case 5:
        {
          ShooterAngle.Set(ControlMode::Position, 6000);
          if(ShooterAngle.GetSelectedSensorPosition()>=6000)
          {
            A = 7;
          }
        }
        break;
        case 7:
        {
          TopShooterPID.SetReference(Lower, rev::ControlType::kVelocity);
          BottomShooterPID.SetReference(Lower, rev::ControlType::kVelocity);
          if(TopShooterEncoder.GetVelocity()>=1500)
          {
            WheelStopper.Set(true);
            Magazine.Set(-.65);
            A = 10;
          }
        }
        break;
        case 10:
        {
          //resets
          LeftDrive1.SetSelectedSensorPosition(0);
          RightDrive1.SetSelectedSensorPosition(0);
          m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
          PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
          A = 20;
          PathTime.Start();
        }
        break;
        //Intake
        case 15:
        {
          IntakePosition.Set(frc::DoubleSolenoid::Value::kForward);
          Intake1.Set(ControlMode::PercentOutput, 1);
          TopShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
          BottomShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
          if(IntakePosition.GetFwdChannel()==1)
          {
            ShooterAngle.Set(ControlMode::Position, -20500);
            A = 17;
          }
        }
        break;
        case 17:
        {
          if(ShooterAngle.GetSelectedSensorPosition()<=-20500)
          {
            if(BackSwitch.Get() == 1 and SideSwitch.Get() == 1)
            {
              //All wheels move
              Magazine.Set(.65);
              WheelStopper.Set(true);
            }
            else if( BackSwitch.Get() == 0 and SideSwitch.Get() == 1)
            {
              //Back two wheels stop
              WheelStopper.Set(false);
              Magazine.Set(.65);
            }
            else if(BackSwitch.Get() == 0 and SideSwitch.Get() == 0)
            {
              Magazine.Set(0);
              WheelStopper.Set(false);
            }
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
            PathTime.Reset();
            PathTime.Start();
            A = 30;
          }
        }
        break;
        case 25:
        {
          ShooterAngle.Set(ControlMode::Position, 6000);
          if(ShooterAngle.GetSelectedSensorPosition()>=6000)
          {
            IntakePosition.Set(frc::DoubleSolenoid::kReverse);
            Intake1.Set(ControlMode::PercentOutput, 0);
          }
        }
        break;
        case 30:
        {
          //moves back to starting position
          const frc::Trajectory::State goal6 = trajectory6.Sample(PathTime.Get());
          frc::ChassisSpeeds adjustedSpeeds6 = PathFollower6.Calculate(FieldPosition, goal6);
          auto [left6, right6] = DriveKin.ToWheelSpeeds(adjustedSpeeds6);
          RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left6));
          LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right6));
          if(PathTime.HasElapsed(trajectory2.TotalTime()+units::time::second_t(.3)))
          {
            PathTime.Stop();
            PathTime.Reset();
            PathTime.Start();
            A = 40;
          }
        }
        break;
        //Shoot
        case 40:
        {
          TopShooterPID.SetReference(Lower, rev::ControlType::kVelocity);
          BottomShooterPID.SetReference(Lower, rev::ControlType::kVelocity);
          if(TopShooterEncoder.GetVelocity()>=1500)
          {
            WheelStopper.Set(true);
            Magazine.Set(-.65);
            A = 50;
          }
        }
        break;
      }
    
  } 
  if(m_autoSelected == kFourBall) //4-5 trajectories depending on where we shoot from
  { 
    switch (C) 
    {
      //Shoot one
      case 10:
      {
        //resets
        LeftDrive1.SetSelectedSensorPosition(0);
        RightDrive1.SetSelectedSensorPosition(0);
        m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
        PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
        C = 20;
        PathTime.Start();
      }
      break;
      //Intake
      //Intake out
      //Intake on
      break;
      case 20:
      {
        //move backwards
        const frc::Trajectory::State goal7 = trajectory7.Sample(PathTime.Get());
        frc::ChassisSpeeds adjustedSpeeds7 = PathFollower7.Calculate(FieldPosition, goal7);
        auto [left7, right7] = DriveKin.ToWheelSpeeds(adjustedSpeeds7);
        RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left7));
        LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right7));
        if(PathTime.HasElapsed(trajectory7.TotalTime()+units::time::second_t(.3)))
        {
          PathTime.Stop();
          PathTime.Reset();
          PathTime.Start();
          C = 30;
        }
      break;
      }
      //Another trajectory to grab other ball
      //Make sure intake is still on

      case 30:
      {
        //moves back to starting position (current trajectory is subject to change since ending position will change)
        const frc::Trajectory::State goal8 = trajectory8.Sample(PathTime.Get());
        frc::ChassisSpeeds adjustedSpeeds8 = PathFollower8.Calculate(FieldPosition, goal8);
        auto [left8, right8] = DriveKin.ToWheelSpeeds(adjustedSpeeds8);
        RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left8));
        LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right8));
        if(PathTime.HasElapsed(trajectory8.TotalTime()+units::time::second_t(.3)))
        {
          PathTime.Stop();
          PathTime.Reset();
          PathTime.Start();
          C = 40;
        }
      }
      break;
      //Shoot 2 in case 40
      //zero everything case 50
      //Turn intake on and out case 60
      //Move to alliance station wall pick up 1 (trajectory) case 70
      // move to shoot (trajectory) case 80
      //shoot 1

    
    }
  }
  if(m_autoSelected == kFiveBall) //5-6 trajectories depending on where we shoot from
  {  
    switch (D) 
    {
      //Shoot
      case 10:
      {
        //resets
        LeftDrive1.SetSelectedSensorPosition(0);
        RightDrive1.SetSelectedSensorPosition(0);
        m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
        PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
        D = 20;
        PathTime.Start();
      }
      break;
      //Intake
      case 20:
      {
        //move backwards
        const frc::Trajectory::State goal9 = trajectory9.Sample(PathTime.Get());
        frc::ChassisSpeeds adjustedSpeeds9 = PathFollower9.Calculate(FieldPosition, goal9);
        auto [left9, right9] = DriveKin.ToWheelSpeeds(adjustedSpeeds9);
        RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left9));
        LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right9));
        if(PathTime.HasElapsed(trajectory9.TotalTime()+units::time::second_t(.3)))
          {
            PathTime.Stop();
            PathTime.Reset();
            PathTime.Start();
            D = 30;
          }
      }
      break;
      case 30:
        {
          //moves back to starting position
          const frc::Trajectory::State goal10 = trajectory10.Sample(PathTime.Get());
          frc::ChassisSpeeds adjustedSpeeds10 = PathFollower10.Calculate(FieldPosition, goal10);
          auto [left10, right10] = DriveKin.ToWheelSpeeds(adjustedSpeeds10);
          RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left10));
          LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right10));
          if(PathTime.HasElapsed(trajectory10.TotalTime()+units::time::second_t(.3)))
          {
            PathTime.Stop();
            PathTime.Reset();
            PathTime.Start();
            D = 40;
          }
        }
        break;
        //Shoot
        //Same thing as four ball but shoot 2 at the end
    }
  }
  else //2-3 trajectories depending on where we shoot from
  {
    // Default Auto goes here (Three ball)
    switch (B)
    {
      //Shoot
      case 10:
      {
        //resets
        LeftDrive1.SetSelectedSensorPosition(0);
        RightDrive1.SetSelectedSensorPosition(0);
        m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
        PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
        B = 20;
        PathTime.Start();
      }
      break;
      //intake
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
          PathTime.Reset();
          PathTime.Start();
          B = 30;
        }
      }
      break;
      //Trajectory to grab other ball
      //Trajectory to move back
      //shoot 2 balls
      //Everything past case 20 need to change
      case 30:
      {
        //moves back to starting position
        const frc::Trajectory::State goal2 = trajectory2.Sample(PathTime.Get());
        frc::ChassisSpeeds adjustedSpeeds2 = PathFollower2.Calculate(FieldPosition, goal2);
        auto [left2, right2] = DriveKin.ToWheelSpeeds(adjustedSpeeds2);
        RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left2));
        LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right2));
        if(PathTime.HasElapsed(trajectory2.TotalTime()+units::time::second_t(.3)))
        {
          PathTime.Stop();
          PathTime.Reset();
          PathTime.Start();
          B = 40;
        }
      }
      break;
      //intake
      case 40:
      {
        //moves to second ball
        const frc::Trajectory::State goal3 = trajectory3.Sample(PathTime.Get());
        frc::ChassisSpeeds adjustedSpeeds3 = PathFollower3.Calculate(FieldPosition, goal3); //PathFollower4
        auto [left3, right3] = DriveKin.ToWheelSpeeds(adjustedSpeeds3);
        RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left3));
        LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right3));
        if(PathTime.HasElapsed(trajectory3.TotalTime()+units::time::second_t(.3)))
        {
          PathTime.Stop();
          PathTime.Reset();
          PathTime.Start();
          B = 45;
        }
      }
      break;
      case 45:
      {
        //resets or rezero's values
        Pigeon.SetFusedHeading(0, 30);
        Pigeon.GetFusedHeading(Status);
        LeftDrive1.SetSelectedSensorPosition(0);
        RightDrive1.SetSelectedSensorPosition(0);
        m_odometry.ResetPosition(frc::Pose2d(units::length::meter_t(0),units::length::meter_t(0),
        PigeonToRotation(Status.heading)),PigeonToRotation(Status.heading));
        B = 50;
        PathTime.Start();
      }
      break;
      case 50:
      {
        //moves to shoot
        const frc::Trajectory::State goal4 = trajectory4.Sample(PathTime.Get());
        frc::ChassisSpeeds adjustedSpeeds4 = PathFollower4.Calculate(FieldPosition, goal4);         
        auto [left4, right4] = DriveKin.ToWheelSpeeds(adjustedSpeeds4);
        RightDrive1.Set(ControlMode::Velocity, ConversionToRaw(left4));
        LeftDrive1.Set(ControlMode::Velocity, ConversionToRaw(right4));
        if(PathTime.HasElapsed(trajectory4.TotalTime()+units::time::second_t(.3)))
        {
          PathTime.Stop();
          PathTime.Reset();
          PathTime.Start();            
          B = 60;
        }
      }
      break;
      //Shoot
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
  frc::SmartDashboard::PutNumber("ClimbAngle1", ClimberAngle1.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("ClimberAngle2", ClimberAngle2.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("ClimberHeight1", Climber1Encoder.GetPosition());
  frc::SmartDashboard::PutNumber("ClimberHeight2", Climber2Encoder.GetPosition());
  frc::SmartDashboard::PutNumber("TopShootSpeed", TopShooterEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("BottomShooterSpeed", BottomShooterEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("ShooterSpeed", ShooterAngle.GetSelectedSensorVelocity());

  //Drive off of Joysticks
  RightDrive1.Set(ControlMode::PercentOutput, Driver.GetRightY()*.65);
  LeftDrive1.Set(ControlMode::PercentOutput, Driver.GetLeftY()*.65);

  //ShooterAngle.Set(ControlMode::PercentOutput, Manipulator.GetRightY());
    
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
  if(Driver.GetRightTriggerAxis()==0 and Driver.GetLeftTriggerAxis()==1 
  and Manipulator.GetAButton()==0 and Manipulator.GetYButton()==0)
  {
    IntakePosition.Set(frc::DoubleSolenoid::Value::kForward);
    Intake1.Set(ControlMode::PercentOutput, 1);
    TopShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
    BottomShooterPID.SetReference(-IntakeSpeed, rev::ControlType::kVelocity);
    if(IntakePosition.GetFwdChannel()==1)
    {
      ShooterAngle.Set(ControlMode::Position, -20500);
    }
  }
  else if(Driver.GetRightTriggerAxis()==1 and Driver.GetLeftTriggerAxis()==0 
  and Manipulator.GetAButton()==0 and Manipulator.GetYButton()==0)
  {
    IntakePosition.Set(frc::DoubleSolenoid::Value::kForward);
    Intake1.Set(ControlMode::PercentOutput, -.5);
  }
  else if (Driver.GetRightTriggerAxis()==0 and Driver.GetLeftTriggerAxis()==0 
  and Manipulator.GetAButton()==1 and Manipulator.GetYButton()==0)
  {
    TopShooterPID.SetReference(Higher, rev::ControlType::kVelocity);
    BottomShooterPID.SetReference(Higher, rev::ControlType::kVelocity);
    ShooterAngle.Set(ControlMode::Position, -6000);
    if(TopShooterEncoder.GetVelocity()>=3000 and ShooterAngle.GetSelectedSensorPosition()<=-6000)
    {
      WheelStopper.Set(false);
      Magazine.Set(-.65);
      if(TopShooterEncoder.GetVelocity()>=3000 and ShooterAngle.GetSelectedSensorPosition()<=-6000)
      {
        WheelStopper.Set(true);
      }
    }
  }
  else if (Driver.GetRightTriggerAxis()==0 and Driver.GetLeftTriggerAxis()==0 
  and Manipulator.GetAButton()==0 and Manipulator.GetYButton()==1)
  {
    TopShooterPID.SetReference(IntakeSpeed, rev::ControlType::kVelocity);
    BottomShooterPID.SetReference(IntakeSpeed, rev::ControlType::kVelocity);
    ShooterAngle.Set(ControlMode::Position, 6000);
    if(TopShooterEncoder.GetVelocity()>=1500 and ShooterAngle.GetSelectedSensorPosition()>=6000)
    {
      WheelStopper.Set(false);
      Magazine.Set(-.65);
      if(TopShooterEncoder.GetVelocity()>=1500 and ShooterAngle.GetSelectedSensorPosition()>=6000)
      {
        WheelStopper.Set(true);
      }
    }
  }
  else
  {
    ShooterAngle.Set(ControlMode::Position, 0);
    Intake1.Set(ControlMode::PercentOutput, 0);
    TopShooter.Set(0);
    BottomShooter.Set(0);
    Magazine.Set(0);
    if(ShooterAngle.GetSelectedSensorPosition()>=0)
    {
      IntakePosition.Set(frc::DoubleSolenoid::Value::kReverse);
    }

  }
    if(Driver.GetLeftTriggerAxis()==1 and BackSwitch.Get() == 1 and SideSwitch.Get() == 1)
      {
        //All wheels move
        Magazine.Set(.65);
        WheelStopper.Set(true);
      }
      else if(Driver.GetLeftTriggerAxis()==1 and BackSwitch.Get() == 0 and SideSwitch.Get() == 1)
      {
        //Back two wheels stop
        WheelStopper.Set(false);
        Magazine.Set(.65);
      }
      else if(Driver.GetLeftTriggerAxis()==1 and BackSwitch.Get() == 0 and SideSwitch.Get() == 0)
      {
        Magazine.Set(0);
        WheelStopper.Set(false);
      }
      else if(Driver.GetLeftTriggerAxis()==1 and BackSwitch.Get() == 0 and SideSwitch.Get() == 0)
      {
        Magazine.Set(0);
        WheelStopper.Set(false);
      }
    
    //4 different positions
    //2 different speeds (lower and higher)
    //Ramp up?
    //A button (Position) and Trigger will set speed
    //Intake
      //Driver axis or bumper depending X
      //Intake position X
      //Intake Speed X
      //Ball holder that goes along with it X
      //limit switches to stop the ball X
    //Shooter
      //Ball holder to move balls to shooter X
      //Shooter position
      //Shooter speed X
      //limit switches to let ball into shooter X
    //climber to be fully programmed
      //Climber on one button + over ride to restart climb or change position
      //Climber on timer = completely autonomous (besides over ride)
      //Hit A button

  /*if(Manipulator.GetRightBumperPressed()==1)
  {

    switch (Hang)
      {
        case 10:
        {
          ClimbRachet.Set(true);
          ClimberAngle1.Set(ControlMode::Position, -10);
          ClimberAngle2.Set(ControlMode::Position, -10);
          Climber1PID.SetReference(-63.3, rev::ControlType::kPosition);
          Climber2PID.SetReference(63.3, rev::ControlType::kPosition);
          Hang = 20;
        }
        break;
        case 20:
        {
          ClimberAngle1.Set(ControlMode::Position, 0);
          ClimberAngle2.Set(ControlMode::Position, 0);
          Hang = 25;
        }
        break;
        case 25:
        {
          Climber1PID.SetReference(0, rev::ControlType::kPosition);
          Climber2PID.SetReference(0, rev::ControlType::kPosition);
          Hang = 30;
        }
        break;
        case 30:
        {
          Climber.Reset();
          Climber.Start();
          ClimberAngle1.Set(ControlMode::Position, 30);
          ClimberAngle1.Set(ControlMode::Position, 30);
          if(Climber.Get()>= units::second_t(3))
          { 
            Climber.Stop();
            Hang = 40;
          }
        }
        break;
        case 40:
        {
          Climber1PID.SetReference(-63.3, rev::ControlType::kPosition);
          Climber2PID.SetReference(63.3, rev::ControlType::kPosition);
          Hang = 50;
        }
        break;
        case 50:
        {
          Climber1Encoder.SetPosition(-2);
          Climber2Encoder.SetPosition(-2);
          Hang = 60;
        }
        break;
        case 60:
        {
          Climber.Reset();
          Climber.Start();
          ClimberAngle1.SetSelectedSensorPosition(30, 0, 10);
          ClimberAngle2.SetSelectedSensorPosition(30, 0, 10);
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
        break;
      }
  }*/
        //One climber hook to shoots up to grab bar
        //Robot to swing back
        //Second climber hook reaches up to grab bar
        //Second climber hook pulls down on bar 
        //First climber hook comes off of bar
        //Angle of robot changes again
        //First climber hook grabs second bar
        //First climber hook pulls down
        // Second climber hook reach for final bar
        //Second climber hook pulls down 
        //Angle of robot changes again
        //First climber hook grabs the final bar
        //First climber hook and second climber hook are on the same level at the same height

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
  const frc::Pose2d crossScale{6_ft, 0_ft, frc::Rotation2d(0_deg)};
  std::vector<frc::Translation2d> interiorWaypoints
    {frc::Translation2d{4_ft, 0_ft},
    frc::Translation2d{5_ft, 0_ft}};
  frc::TrajectoryConfig config{12_fps, 8_fps_sq};
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
  const frc::Pose2d crossScale{6_ft, 0_ft, frc::Rotation2d(0_deg)};
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
