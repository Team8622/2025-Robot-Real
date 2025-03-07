// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.commands.AlgaeAnalog;
import frc.robot.commands.ChainAnalog;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.IntakeAnalog;
import frc.robot.commands.ManualControl;
import frc.robot.subsystems.Algae_Intake;
import frc.robot.subsystems.Coral_Intake;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSimulator; // Add this import statement
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	// Buttons for the Driver Controller

	//public static final DriveSubsystem m_driveTrain = new DriveSubsystem();
	public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
	public static final SwerveSimulator m_sim = new SwerveSimulator(drivebase);
	public static final Coral_Intake m_intake = new Coral_Intake();
	public static final Algae_Intake m_algae = new Algae_Intake();
	public static final Elevator m_chain = new Elevator();
	// public static final ColorSensor m_colorSensor = new ColorSensor();
	final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
	final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);

	// public static final PhotonCameraWrapper m_PhotonCam = new
	// PhotonCameraWrapper();

	public static final CommandXboxController driverXbox = new CommandXboxController(0);
	public static final CommandXboxController controllerXbox = new CommandXboxController(1);

	public final SendableChooser<Command> m_chooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		m_chooser = AutoBuilder.buildAutoChooser();
		DriverStation.silenceJoystickConnectionWarning(true);
		SmartDashboard.putData(m_chooser);
		// registering pathplanner commands
		NamedCommands.registerCommand("coralExtake", new GenericCommand(m_intake, IntakeConstants.outSpeed));
		NamedCommands.registerCommand("coralIntake", new GenericCommand(m_intake, IntakeConstants.inSpeed));
		NamedCommands.registerCommand("coralStop", new GenericCommand(m_intake, 0));
		NamedCommands.registerCommand("algaeIntake", new GenericCommand(m_algae, AlgaeConstants.vacuum));
		NamedCommands.registerCommand("algaeExtake", new GenericCommand(m_algae, AlgaeConstants.spitup));
		NamedCommands.registerCommand("algaeStop", new GenericCommand(m_algae, 0));
		NamedCommands.registerCommand("elevatorBottom", new GenericCommand(m_chain, 0));
		NamedCommands.registerCommand("elevatorL1", new GenericCommand(m_chain, 1));
		NamedCommands.registerCommand("elevatorL2", new GenericCommand(m_chain, 2));
		NamedCommands.registerCommand("elevatorL3", new GenericCommand(m_chain, 3));
		NamedCommands.registerCommand("elevatorL4", new GenericCommand(m_chain, 4));

		// Configure the button bindings
		configureButtonBindings();
		drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
	}

	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> driverXbox.getLeftY() * -1,
			() -> driverXbox.getLeftX() * -1)
			.withControllerRotationAxis(driverXbox::getRightX)
			.deadband(DriveConstants.deadband)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);
	/**
	 * Clone's the angular velocity input stream and converts it to a fieldRelative
	 * input stream.
	 */
	SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
			driverXbox::getRightY)
			.headingWhile(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a robotRelative
	 * input stream.
	 */
	SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
			.allianceRelativeControl(false);

	Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
	Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

	// SmartDashboard command selecter
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return m_chooser.getSelected();
	}

	public void init() {
		m_intake.init();
		m_algae.init();
		m_chain.init();
		//m_chain.homeElevator();
		// solenoid.set(DoubleSolenoid.Value.kReverse);
		// compressor.enableHybrid(80,120);
		// m_colorSensor.init();
	}

	private void configureButtonBindings() {
		// buttons assuming the setpoints are tuned
		// operator buttons
		controllerXbox.a().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.inSpeed)); // green (1) -> manny intake
		controllerXbox.b().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.outSpeed)); // red (2) -> manny extake
		controllerXbox.y().whileTrue(new AlgaeAnalog(m_algae, AlgaeConstants.vacuum)); // yellow (3) -> manny LAUNCH CUBE
		controllerXbox.x().whileTrue(new AlgaeAnalog(m_algae, AlgaeConstants.spitup)); // blue (4) -> wrist deposit
		// controllerXbox.leftTrigger().whileTrue(new ChainAnalog(m_chain, -1));
		// controllerXbox.rightTrigger().whileTrue(new ChainAnalog(m_chain, 1));
		controllerXbox.leftTrigger().whileTrue(new ManualControl(m_chain, -.15));
		controllerXbox.rightTrigger().whileTrue(new ManualControl(m_chain, .15));
		//controllerXbox.leftBumper().whileTrue(new HomeElevator(m_chain));
		controllerXbox.leftBumper().whileTrue(new ManualControl(m_chain, -.1));
		controllerXbox.rightBumper().whileTrue(new ManualControl(m_chain, .1));
		controllerXbox.rightStick().whileTrue(new ManualControl(m_chain, 0));
		// driver buttons
		// sad losers, only having three buttons. I have so many. I am so powerful.
		driverXbox.leftTrigger().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.inSpeed)); // right trigger is in
		driverXbox.leftTrigger().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.outSpeed)); // left trigger is ou
	}
}