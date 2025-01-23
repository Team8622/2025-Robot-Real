// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.FlipConstants;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AlgaeAnalog;
import frc.robot.commands.ChainAnalog;
//import frc.robot.commands.GondolaState;
import frc.robot.commands.IntakeAnalog;
import frc.robot.commands.auton.Full.DriveBackward;
//import frc.robot.commands.auton.Full.DoNothing;
import frc.robot.commands.auton.Full.DriveForward;
import frc.robot.commands.auton.Full.DriveForward2;
import frc.robot.commands.auton.Full.DriveForwardOnly;
import frc.robot.commands.auton.support.AutoBalanceCommand;
import frc.robot.subsystems.Algae_Intake;
import frc.robot.subsystems.Coral_Intake;
//import edu.wpi.first.wpilibj.I2C.Port;
//import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton
//import frc.robot.commands.auton.;
//import frc.robot.commands.auton.Full.DriveForward;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorChain;





/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //Buttons for the Driver Controller


  public static final Coral_Intake m_intake = new Coral_Intake();
   public static final Algae_Intake m_algae = new Algae_Intake();
  public static final ElevatorChain m_chain = new ElevatorChain();
  //public static final ColorSensor m_colorSensor = new ColorSensor();
  public static final DriveSubsystem m_driveTrain = new DriveSubsystem();
  final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
  
  
  //public static final PhotonCameraWrapper m_PhotonCam = new PhotonCameraWrapper();

  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController controller = new CommandXboxController(1);
  
  public static final Trigger strumUp = new POVButton(controller.getHID(), 0);
  public static final Trigger strumDown = new POVButton(controller.getHID(), 180);
  
  public static final BooleanSupplier wham = () -> controller.getRightX() > 0;
  public static final Trigger whammy = new Trigger(wham);
  public static final Bumper pneumatics = new Bumper();
//JoystickButton turnOnButton = new JoystickButton(controller, 3);

  public final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
//JoystickButton turnOnButton = new JoystickButton(controller, 3);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    configureButtonBindings();
    // Configure the button bindings
   
  }
  
   public Command getAutonomousCommand(){
     return m_chooser.getSelected();
   }
  public void init() {
    m_intake.init();
    m_chain.init();
    m_algae.init();
    //solenoid.set(DoubleSolenoid.Value.kReverse);
    //compressor.enableHybrid(80,120);
    //m_colorSensor.init();
  }


  

 

  private void configureButtonBindings(){
    //buttons assuming the setpoints are tuned
    //operator buttons
     

    

    /* AUTO COMMANDS 
     * 
     * 
     * 
    */

    //Just mobility
    final Command kMobility = new SequentialCommandGroup(
      new DriveForwardOnly(m_driveTrain));
    
    //Just Score on speaker 
    /*final Command speakerShot = new SequentialCommandGroup(
      new IntakeAnalog(m_intake, IntakeConstants.inSpeed).withTimeout(0.5), 
      new ShooterAnalog(m_kobe, ShooterConstants.longRange3).withTimeout(1)); */

    final Command speakerDub = new SequentialCommandGroup(
      //new ShootBrother(m_intake, m_kobe)
    );

    /*final Command twoNoteAuto = new SequentialCommandGroup(
      //new ShootBrother(m_intake, m_kobe).withTimeout(2.5),
      //new DriveForward(m_driveTrain, m_intake),
      //new DriveBackward(m_driveTrain, m_intake),
      //new ShootBrother(m_intake, m_kobe));

    /*final Command sideOneNote = new SequentialCommandGroup(
      //new ShootBrother(m_intake, m_kobe).withTimeout(2.5),
      //new DriveForward2(m_driveTrain, m_intake));  
   
   
   Puts options on driver station
   m_chooser.setDefaultOption("Absolutely Nothing",null);
   m_chooser.addOption("Speaker and Nothing", speakerDub);
   m_chooser.addOption("Two Note from Center", twoNoteAuto);
   m_chooser.addOption("Side One note", sideOneNote);
   m_chooser.addOption("Back up", kMobility);
   
   SmartDashboard.putData(m_chooser);*/


    controller.a().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.inSpeed)); //green (1) -> manny intake
    controller.b().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.outSpeed)); //red (2) -> manny extake
    controller.y().whileTrue(new AlgaeAnalog(m_algae, AlgaeConstants.vacuum)); //yellow (3) -> manny LAUNCH CUBE
    controller.x().whileTrue(new AlgaeAnalog(m_algae, AlgaeConstants.spitup)); //blue (4) -> wrist deposit
    controller.leftTrigger().whileTrue(new ChainAnalog(m_chain, ElevatorConstants.up));

    controller.rightTrigger().whileTrue(new ChainAnalog(m_chain, ElevatorConstants.down));
    
    //driver.y().onTrue(new ShooterAnalog(m_kobe, ShooterConstants.layup));
    //driver.x().onTrue(new ShooterAnalog(m_kobe, 0));
    


    //controller.leftBumper().onTrue(kWristNeutral.withTimeout(0.84).andThen(kWristPickup)); //orange (5) -> wrist pickup

   
      

    //driver buttons
    //sad losers, only having three buttons. I have so many. I am so powerful.
    driver.leftTrigger().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.inSpeed)); //right trigger is in
    driver.leftTrigger().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.outSpeed)); //left trigger is out
    //driver.a().onTrue(new FlipReset(m_flip));

    //driver.rightBumper().onTrue(new GearShift());
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   //return m_autoCommand;
  // }

// public Command getAutonomousCommand() {
//   // 1. Create trajectory settings
//   TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//     Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//             .setKinematics(Constants.DriveConstants.kDriveKinematics);

// // 2. Generate trajectory
// Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//     new Pose2d(0, 0, new Rotation2d(0)),
//     List.of(
//             new Translation2d(1, 0),
//             new Translation2d(1, -1)),
//     new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
//     trajectoryConfig);

// // 3. Define PID controllers for tracking trajectory
// PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
// PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
// ProfiledPIDController thetaController = new ProfiledPIDController(
//     Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
// thetaController.enableContinuousInput(-Math.PI, Math.PI);

// // 4. Construct command to follow trajectory
// SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//     trajectory,
//     new Pose2d(0.0,0.0,new Rotation2d(0.0)),
//     Constants.DriveConstants.kDriveKinematics,
//     xController,
//     yController,
//     thetaController,
//     m_driveTrain::setModuleStates,
//     m_driveTrain);
    
// return swerveControllerCommand;
// // 5. Add some init and wrap-up, and return everything
// // return new SequentialCommandGroup(
// //     new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
// //     swerveControllerCommand,
// //     new InstantCommand(() -> swerveSubsystem.stopModules()));
// // }
// }}
}