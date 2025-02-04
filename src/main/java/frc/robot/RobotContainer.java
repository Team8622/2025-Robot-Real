// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AlgaeAnalog;
import frc.robot.commands.ChainAnalog;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.IntakeAnalog;
import frc.robot.subsystems.Algae_Intake;
import frc.robot.subsystems.Coral_Intake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

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

	public static final DriveSubsystem m_driveTrain = new DriveSubsystem();
	public static final Coral_Intake m_intake = new Coral_Intake();
	public static final Algae_Intake m_algae = new Algae_Intake();
	public static final Elevator m_chain = new Elevator();
	// public static final ColorSensor m_colorSensor = new ColorSensor();
	final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
	final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);

	// public static final PhotonCameraWrapper m_PhotonCam = new
	// PhotonCameraWrapper();

	public static final CommandXboxController driver = new CommandXboxController(0);
	public static final CommandXboxController controller = new CommandXboxController(1);

	public static final Trigger strumUp = new POVButton(controller.getHID(), 0);
	public static final Trigger strumDown = new POVButton(controller.getHID(), 180);

	public static final BooleanSupplier wham = () -> controller.getRightX() > 0;
	public static final Trigger whammy = new Trigger(wham);
	public static final Bumper pneumatics = new Bumper();
	// JoystickButton turnOnButton = new JoystickButton(controller, 3);

	public final SendableChooser<Command> m_chooser;

	// JoystickButton turnOnButton = new JoystickButton(controller, 3);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		m_chooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(m_chooser);
		// registering pathplanner commands
		NamedCommands.registerCommand("coralExtake", new GenericCommand(m_intake, IntakeConstants.outSpeed));
		NamedCommands.registerCommand("coralIntake", new GenericCommand(m_intake, IntakeConstants.inSpeed));
		NamedCommands.registerCommand("coralStop", new GenericCommand(m_intake, 0));
		NamedCommands.registerCommand("algaeIntake", new GenericCommand(m_algae, AlgaeConstants.vacuum));
		NamedCommands.registerCommand("algaeExtake", new GenericCommand(m_algae, AlgaeConstants.spitup));
		NamedCommands.registerCommand("algaeStop", new GenericCommand(m_algae, 0));

		// Configure the button bindings
		configureButtonBindings();
	}

	// Returns subsystem methods as COMMANDS for pathplanner use
	// CORAL
	public Command coralExtake() {
		return m_intake.startCommand(IntakeConstants.outSpeed);
	}

	public Command coralIntake() {
		return m_intake.startCommand(IntakeConstants.inSpeed);
	}

	public Command coralStop() {
		return m_intake.stopCommand();
	}

	// ALGAE
	public Command algaeIntake() {
		return m_algae.startCommand(AlgaeConstants.vacuum);
	}

	public Command algaeExtake() {
		return m_algae.startCommand(AlgaeConstants.spitup);
	}

	public Command algaeStop() {
		return m_algae.stopCommand();
	}

	// ELEVATOR
	public Command elevatorL1() {
		return m_chain.setLevelCommand(1);
	}

	public Command elevatorL2() {
		return m_chain.setLevelCommand(2);
	}

	public Command elevatorL3() {
		return m_chain.setLevelCommand(3);
	}

	public Command elevatorL4() {
		return m_chain.setLevelCommand(4);
	}

	public Command elevatorBottom() {
		return m_chain.setLevelCommand(0);
	}

	// SmartDashboard command selecter
	public Command getAutonomousCommand() {
		return m_chooser.getSelected();
	}

	public void init() {
		m_intake.init();
		m_algae.init();
		m_chain.homeElevator();
		// solenoid.set(DoubleSolenoid.Value.kReverse);
		// compressor.enableHybrid(80,120);
		// m_colorSensor.init();
	}

	private void configureButtonBindings() {
		// buttons assuming the setpoints are tuned
		// operator buttons
		controller.a().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.inSpeed)); // green (1) -> manny intake
		controller.b().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.outSpeed)); // red (2) -> manny extake
		controller.y().whileTrue(new AlgaeAnalog(m_algae, AlgaeConstants.vacuum)); // yellow (3) -> manny LAUNCH CUBE
		controller.x().whileTrue(new AlgaeAnalog(m_algae, AlgaeConstants.spitup)); // blue (4) -> wrist deposit
		controller.leftTrigger().whileTrue(new ChainAnalog(m_chain, m_chain.elevatorLevel - 1));
		controller.rightTrigger().whileTrue(new ChainAnalog(m_chain, m_chain.elevatorLevel + 1));
		controller.leftBumper().whileTrue(new HomeElevator(m_chain));

		// driver buttons
		// sad losers, only having three buttons. I have so many. I am so powerful.
		driver.leftTrigger().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.inSpeed)); // right trigger is in
		driver.leftTrigger().whileTrue(new IntakeAnalog(m_intake, IntakeConstants.outSpeed)); // left trigger is ou
	}
}