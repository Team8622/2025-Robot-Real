package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.system.plant.LinearSystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSimSubsystem extends SubsystemBase {
    private final SparkMax simulatedMotor;
    private final RelativeEncoder simulatedEncoder;
    private final PIDController pidController;
    private final ElevatorFeedforward feedforward;
    private final Joystick joystick;

    // Elevator Simulation Model
    private final ElevatorSim elevatorSim;

    // Simulated state variables
    private double simulatedHeightMeters = 0.0;
    private double simulatedVelocityMetersPerSecond = 0.0;
    
    public ElevatorSimSubsystem() {
        // Simulated motor setup
        simulatedMotor = new SparkMax(ElevatorConstants.elevatorLeadSim, MotorType.kBrushless);
        simulatedEncoder = simulatedMotor.getEncoder();
        joystick = new Joystick(2);

        // PID and Feedforward for control
        pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

        // Create elevator physics model

        elevatorSim = new ElevatorSim(
            // LinearSystemId.identifyPositionSystem(ElevatorConstants.kV, ElevatorConstants.kA), // System model
            DCMotor.getNEO(2), // 2 NEO motors
            ElevatorConstants.gearRatio, // Gear ratio
            100,
            Units.inchesToMeters(ElevatorConstants.drumDiameter/2), // Drum radius
            0.0, // Min height (meters)
            Units.inchesToMeters(ElevatorConstants.maxPos), // Max height (meters)
            true,
            0 
             // Simulate gravity
         // Initial position
        );
    }

    @Override
    public void periodic() {
        // Update SmartDashboard with real-world values (if running on real robot)
        SmartDashboard.putNumber("Simulated Elevator Height (m)", simulatedHeightMeters);
        SmartDashboard.putNumber("Simulated Elevator Velocity (m/s)", simulatedVelocityMetersPerSecond);
    }

    @Override
    public void simulationPeriodic() {
        // Get joystick input (assuming Y-axis controls the elevator)
        double joystickInput = joystick.getY();  // Assuming Y-axis controls the elevator (inverted if necessary)
    
        // Map joystick input to motor voltage (scale to [-12, 12] volts)
        double voltage = joystickInput * 12.0;  // Scale joystick input (-1 to 1) to motor voltage (-12V to 12V)
    
        // Apply the voltage input to the simulation (this should influence the simulated motor)
        elevatorSim.setInputVoltage(voltage);
    
        // Update the simulation (with a 20ms time step)
        double dt = 0.02; // 20 ms loop time for the simulation
        elevatorSim.update(dt);
    
        // Update simulated height and velocity from the simulation model
        simulatedHeightMeters = elevatorSim.getPositionMeters();  // Get the current height in meters
        simulatedVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();  // Get velocity in meters per second
    
        // Convert height in meters to encoder counts and update encoder position
        double heightInches = Units.metersToInches(simulatedHeightMeters);  // Convert height to inches
        double encoderPosition = heightInches * ElevatorConstants.countsPerInch;  // Convert to encoder counts
        simulatedEncoder.setPosition(encoderPosition);  // Set the encoder position based on the simulated height
    }
    

    public void setElevatorPosition(double heightMeters) {
        double pidOutput = pidController.calculate(simulatedHeightMeters, heightMeters);
        double ffOutput = feedforward.calculate(simulatedVelocityMetersPerSecond);

        double motorOutput = pidOutput + ffOutput;

        // Send power to the simulated motor
        simulatedMotor.set(motorOutput);

        // Apply input voltage to simulation
        elevatorSim.setInputVoltage(motorOutput * 12.0);
    }
}
