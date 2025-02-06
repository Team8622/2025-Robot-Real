// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
//import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  // Driving encoder uses the integrated FX encoder
  // e.g. testMotor.getSelectedSensorPosition();

  SparkMax m_driveMotor;
  SparkMax m_turningMotor;
  DutyCycleEncoder m_turnEncoder;
  // PID controller for velocity. DO NOT SET kD.  It is redundant as setVoltage() already controls this
  private final PIDController m_drivePIDController =
      new PIDController(
          ModuleConstants.kPModuleDriveController,
          ModuleConstants.kIModuleDriveController, // 0
          ModuleConstants.kDModuleDriveController // 0
      );

  // ShuffleboardTab PIDtab = Shuffleboard.getTab("PID Tuning");


  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turnPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurnController,
          ModuleConstants.kIModuleTurnController, // 0
          ModuleConstants.kDModuleTurnController,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  
  // In the pose example.
  // NOTE: The passed-in gains must have units consistent with the distance units, or a compile-time error will be thrown.
  // kS should have units of volts, kV should have units of volts * seconds / distance, and kA should have units of volts * seconds^2 / distance.
//  // Gains are for example purposes only - must be determined for your own robot!
//  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
//  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
      DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

  SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
      DriveConstants.ksTurning, DriveConstants.kvTurning, DriveConstants.kaTurning);

  double angleOffest = 0;

  // shuffleboard stuff
  ShuffleboardLayout shuffleboardContainer;

  /**
   * Constructs a swerve module
   * @param driveMotorChannel ID of the drive motor
   * @param turningMotorChannel ID of the turn motor
   * @param turningEncoderChannel ID of the CANCoder
   * @param angleZero CANCoder offset
   * @param encoderReversed is the turn encoder reversed
   * @param driveReversed is the drive motor reversed
   * @param container shuffleboard container to print debug to
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double angleZero,
      boolean encoderReversed,
      boolean driveReversed,
      ShuffleboardLayout container
      ) {

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(35);

        m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

        m_turningMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //presetEncoders();
        m_turnEncoder = new DutyCycleEncoder(turningEncoderChannel, 1, 0);
        //m_turnPIDController.enableContinuousInput(-180.0, 180.0);
        // m_turnEncoder.setDistancePerRotation(1);
        //m_turnEncoder.setPositionOffset(angleZero/360);
        m_turnPIDController.enableContinuousInput(-Math.PI,Math.PI);
        
        angleOffest = angleZero;
        shuffleboardContainer = container;
        
      }
      
  /**
   * Gets the heading of the module
   * @return the absolute position of the CANCoder
   */
  public double getModuleHeading(){
    return getEncoderRadians();
    
  }
  public double getEncoderRadians(){
    return (Math.PI*2)*(MathUtil.inputModulus(m_turnEncoder.get()-(angleOffest/360), -0.5, 0.5))*-1;
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double m_speedMetersPerSecond =
        ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getEncoder().getVelocity();

    return new SwerveModuleState(m_speedMetersPerSecond, new Rotation2d(getEncoderRadians()));
  }
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      m_driveMotor.getEncoder().getPosition(), new Rotation2d(getEncoderRadians())
    );
  }


  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    double m_speedMetersPerSecond =
        ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getEncoder().getVelocity();

        
    // Optimize the reference state to avoid spinning further than 90 degrees
    state.optimize(new Rotation2d(getEncoderRadians()));
    
//    stop the code to stop it from moving if the speed is very, very small
    if (Math.abs(state.speedMetersPerSecond) <= 1){
      m_turningMotor.set(0);
      m_driveMotor.set(0);
      return;
    }

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_speedMetersPerSecond, state.speedMetersPerSecond);
            // + driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turnPIDController.calculate(getEncoderRadians(), state.angle.getRadians())
            /*+ 
            

            
            .calculate(m_turnPIDController.getSetpoint().velocity)*/;
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
    
  }
  // public SwerveModuleState getState(){
  //   return new SwerveModuleState(getCANCoder(),new Rotation2d(m_turnEncoder.getDistance()))
  // }

  

  @Deprecated
  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
      //m_turnEncoder.reset();
      m_driveMotor.getEncoder().setPosition(0);
  }
      
  public void periodic_func() {
    //SmartDashboard.putNumber(shuffleboardContainer.getTitle() + " ABS", m_turnEncoder.getAbsolutePosition());
    //SmartDashboard.putNumber(shuffleboardContainer.getTitle() + " pos", m_turnEncoder.getDistance());
  }


      }