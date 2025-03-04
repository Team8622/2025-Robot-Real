// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CANConstants {
		public static final int intakeMain = 13;
        public static final int intakeSushi = 14;
        public static final int algaeLead = 11;
        public static final int algaeFollow = 12;
        public static final int armsMain = 15;
        public static final int armsFollow = 16;
    }

    public static final class ElevatorConstants{
        //motor CAN IDS and other ports
        public static final int elevatorLead = 9;
        public static final int elevatorFollow = 10;
        public static final int limitSwitchPort = 5;
        //all units should be in inches
        public static final double distanceFromGround = 6; //placeholder, measure distance from motor to the ground in inches
        public static final double L1 = 18 - distanceFromGround;
        public static final double L2 = 32 - distanceFromGround;
        public static final double L3 = 47.5 - distanceFromGround;
        public static final double L4 = 72 - distanceFromGround;
        
        public static double bottomPos = 0;
        public static double minPos = 0;
        public static double maxPos = L4;
        public static double posTolerance = .5;

        public static double maxVelocity = 60; // inches per second
        public static double maxAcceleration = 60;
        public static double max_output = 1; //percent of motor power

        // PID tuning values. this is going to require a lot of testing to fine tune. This is the foundation of the elevator PID loop
        public static double kP = 0.056; // full output until there is an error of 18 inches
        public static double kI = 0.019; // corrects any accumulated error in kP
        public static double kD = 0.022; //smooths out oscillations
        public static double kS = .2; // static friction ff (feedforward)
        public static double kG = 0.4; // gravity ff
        public static double kV = 0.19; // velocity ff - anticipates expected velocity

        // We need to find the counts per revolution of the 
        // encoders on the elevator motors. 16 is a placeholder for now.
        public static final int countsPerRevolution = 42;
        public static final int gearRatio = 40; // placeholder
        public static final double drumDiameter = 2;
        public static final double drumCircumference = Math.PI * drumDiameter;
        public static final double countsPerInch = (countsPerRevolution * gearRatio)/drumCircumference;
        // ^ this used to convert the encoder information to inches
    }
    

    public static final class IntakeConstants {
        public static final double inSpeed = -.25;
        public static final double outSpeed = .25;


    }

    public static final class AlgaeConstants {
        public static final double vacuum = .3;
        public static final double spitup = -0.3;
    }


    
    public static final class DriveConstants {
        //not important rn
        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
         public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
         public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
         public static final double MAX_SPEED  = Units.feetToMeters(14.5);

        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kRearLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kRearRightDriveMotorPort = 8;
    
        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kRearLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kRearRightTurningMotorPort = 7;

    
        public static final int kFrontLeftTurningEncoderPort = 1;
        public static final int kRearLeftTurningEncoderPort = 2;
        public static final int kFrontRightTurningEncoderPort = 3;
        public static final int kRearRightTurningEncoderPort = 0;
    

        public static final double kFrontLeftAngleZero = 123.3;//was 45 thn 20
        //123.3, -177,65,84.5
        public static final double kRearLeftAngleZero = -177;//was 10
        public static final double kFrontRightAngleZero =65; //40
        public static final double kRearRightAngleZero = 84.5;//was 100, then 125 
    
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kRearLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kRearRightTurningEncoderReversed = false;

    
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kRearRightDriveEncoderReversed = true; //was true

        public static final double kTrackWidth = 0.5588;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.56515;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //left front  CURRENTLY:(+,-), IDEA:(-,+), WPILIB:(+,+)
                new Translation2d(-kWheelBase / 2,-kTrackWidth / 2), //right front  CURRENTLY:(+,+), IDEA:(-,-), WPILIB:(+,-)
                new Translation2d(kWheelBase / 2,kTrackWidth / 2), //left back   CURRENTLY:(-,-), IDEA:(+,+), WPILIB:(-,+)
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2) //right back   CURRENTLY:(-,+), IDEA:(+,-), WPILIB:(-,-)
                );
    

        public static final boolean kGyroReversed = true;
    
    
        // Values to scale joystick inputs to desired states.
        public static double kMaxSpeedMetersPerSecond = 4.5; // LOCKED IN AT 4.5
        public static final double kMaxRotationalSpeed =
            2 * Math.PI; //3*pi
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        public static final double ksVolts = 0.73394;
        public static final double kvVoltSecondsPerMeter = 2.4068;
        public static final double kaVoltSecondsSquaredPerMeter = 0.28749;
    
        public static final double ksTurning = 0; // LOCKED IN!  -----  old 0.66202
        public static final double kvTurning = 0; //0.7x5 // 3.0052
        public static final double kaTurning = 0; // Default to zero
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
        public static double kPDriveVel;
        public static final double deadband = 0.05;

      }
    
      public static final class ModuleConstants {
    
        public static final double kDriveGearRatio = 7.13;
    

        public static final double kPModuleTurnController =  .8;
        ; //8.3 // TUNE: 8.2142
        public static final double kIModuleTurnController = 0; // DO NOT USE
        public static final double kDModuleTurnController = 0; // TUNE
    
        // Acceleration could be 8pi to make module get anywhere in 0.5 seconds.
        // Will never reach max velocity, so it can be right at the "top" of the triangle.
        // In this case, that would be 2pi.
    
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 6 * Math.PI;
    
        public static final double kPModuleDriveController = .2; // TUNE
        public static final double kIModuleDriveController = 0; // DO NOT USE
        public static final double kDModuleDriveController = 0;
    
    
        public static final int kDriveFXEncoderCPR = 40;
        public static final int kTurningEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.1016; // 4 inches
        public static final double kWheelCircumferenceMeters =
            kWheelDiameterMeters * Math.PI; // C = D * pi
        public static final double kDrivetoMetersPerSecond =
            (10 * kWheelCircumferenceMeters) / (kDriveGearRatio * 2048);
      }

      public static final class AutoConstants{

        public static final double kMaxSpeedMetersPerSecond = 20 / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;
        public static final double kAutoAlignP = 0.1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    };
    public static final class VisionConstants{
        public static final String cameraName = "AprilTagCamera";
        public static final Transform3d robotToCam = 
            new Transform3d(
                new Translation3d(-0.3175,0,0.381),
                new Rotation3d(0,0,(Math.PI))
            );
        public static final double TARGET_HEIGHT_METERS = 0.159;
        public static final double CAMERA_HEIGHT_METERS = .4826;
    };
}
