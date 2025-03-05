package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends GenericSubsystem {
    private SparkMax primaryMotor;
    private SparkMax followerMotor;
    private RelativeEncoder encoder;
    private DigitalInput bottomLimit;
    private PIDController pidController;
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile profile;

    private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
    private boolean isHomed = false;
    private double setpoint = 0.0;
    SparkMaxConfig leadConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    double currentPos;
    public int elevatorLevel = 0;

    public enum ElevatorPosition {
        DOWN(0),
        POSITION_1(ElevatorConstants.L1),
        POSITION_2(ElevatorConstants.L2),
        POSITION_3(ElevatorConstants.L3),
        POSITION_4(ElevatorConstants.L4);

        public final double positionInches;

        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }

        public double getPositionInches() {
            return positionInches;
        }
    }

    public Elevator() {
    }

    public void init(){
        
        isHomed = true; //TODO: Temporary until limit switch works lol
        primaryMotor = new SparkMax(ElevatorConstants.elevatorLead, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.elevatorFollow, MotorType.kBrushless);
        // done using rev hardware client
        
        followerConfig.follow(9, true);
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.smartCurrentLimit(50);
        followerConfig.voltageCompensation(12.0);
        // Configure follower
        //followerMotor.configure(followerConfig, null, null);
        leadConfig.idleMode(IdleMode.kBrake);
        leadConfig.smartCurrentLimit(50);
        leadConfig.voltageCompensation(12.0);

        encoder = primaryMotor.getEncoder();
        bottomLimit = new DigitalInput(ElevatorConstants.limitSwitchPort);

        constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.maxVelocity,
                ElevatorConstants.maxAcceleration);

        pidController = new PIDController(
                ElevatorConstants.kP,
                ElevatorConstants.kI,
                ElevatorConstants.kD);

        pidController.setTolerance(0.5); // 0.5 inches position tolerance

        // Initialize states and profile
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);

        configureMotors();
    }
    private void configureMotors() {
        // Primary motor configuration
        primaryMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower motor configuration
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        primaryMotor.set(-1); //TODO: temporary
        currentPos = encoder.getPosition() / ElevatorConstants.countsPerInch;

        // Calculate the next state and update current state
        currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

        if (bottomLimit.get()) {
            handleBottomLimit();
        }

        if (getHeightInches() > ElevatorConstants.maxPos) {
            stopMotors();
        }

        // Only run control if homed
        if (isHomed) {
            double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
            double ff = calculateFeedForward(currentState);

            double outputPower = MathUtil.clamp(
                    pidOutput + ff,
                    -ElevatorConstants.max_output,
                    ElevatorConstants.max_output);

            SmartDashboard.putNumber("PID Output", pidOutput);
            SmartDashboard.putNumber("Feedforward", ff);
            SmartDashboard.putNumber("Output Power", outputPower);
            //primaryMotor.set(outputPower);
        }

        // Update SmartDashboard
        updateTelemetry();
    }

    private void handleBottomLimit() {
        //stopMotors();
        encoder.setPosition(ElevatorConstants.bottomPos * ElevatorConstants.countsPerInch);
        isHomed = true;
        setpoint = ElevatorConstants.bottomPos;
        currentState = new TrapezoidProfile.State(ElevatorConstants.bottomPos, 0);
        goalState = new TrapezoidProfile.State(ElevatorConstants.bottomPos, 0);
        pidController.reset();
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    public boolean isAtHeight(double targetHeightInches) {
        // Check if the elevator is within a small tolerance of the target height
        return pidController.atSetpoint() &&
                Math.abs(getHeightInches() - targetHeightInches) < ElevatorConstants.posTolerance;
    }

    private double calculateFeedForward(TrapezoidProfile.State state) {
        // kS (static friction), kG (gravity), kV (velocity),
        return ElevatorConstants.kS * Math.signum(state.velocity) +
                ElevatorConstants.kG +
                ElevatorConstants.kV * state.velocity;
    }

    public void setPositionInches(double inches) {
        if (!isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        setpoint = MathUtil.clamp(
                inches,
                ElevatorConstants.minPos,
                ElevatorConstants.maxPos);

        // Update goal state for motion profile
        goalState = new TrapezoidProfile.State(setpoint, 0);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Target", setpoint);
        SmartDashboard.putBoolean("Elevator Homed", isHomed);
        SmartDashboard.putString("Elevator State", currentTarget.toString());
        SmartDashboard.putNumber("Elevator Current", primaryMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", currentState.velocity);
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
    }

    public double getHeightInches() {
        return encoder.getPosition() / ElevatorConstants.countsPerInch;
    }

    public void homeElevator() {
        primaryMotor.set(-0.1); // Slow downward movement until bottom limit is hit
        if (bottomLimit.get()) {
            handleBottomLimit();
        }
    }

    public boolean isAtPosition(ElevatorPosition position) {
        return pidController.atSetpoint() &&
                Math.abs(getHeightInches() - position.positionInches) < 0.5;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public ElevatorPosition getCurrentTarget() {
        return currentTarget;
    }

    public void setManualPower(double power) {
        // Disable PID control when in manual mode
        pidController.reset();
        currentState = new TrapezoidProfile.State(getHeightInches(), 0);
        goalState = new TrapezoidProfile.State(getHeightInches(), 0);

        if (!isHomed && power < 0) {
            power = 0;
        }

        if (getHeightInches() >= ElevatorConstants.maxPos && power > 0) {
            power = 0;
        }

        if (bottomLimit.get() && power < 0) {
            power = 0;
        }

        primaryMotor.set(MathUtil.clamp(power, -ElevatorConstants.max_output, ElevatorConstants.max_output));
    }
    public void start (int level) {
        this.setLevel(level);
    }

    public void adjustLevel(int delta) {
        setLevel(MathUtil.clamp(elevatorLevel + delta, 0, 4));
    }

    public void setLevel(int level) {
        // Array of elevator positions in inches corresponding to each level
        ElevatorPosition[] levels = {
                ElevatorPosition.DOWN,
                ElevatorPosition.POSITION_1,
                ElevatorPosition.POSITION_2,
                ElevatorPosition.POSITION_3,
                ElevatorPosition.POSITION_4
        };
        // Ensure the level is within the valid range (1 to 4)
        if (level >= 1 && level <= 4) {
            setPositionInches(levels[level].getPositionInches()); // Convert level to index (1 -> index 1)
            elevatorLevel = level;
            currentTarget = levels[level];
        } else {
            // Default to bottom position if the level is not valid
            setPositionInches(levels[0].getPositionInches()); // Use index 0 (bottom)
            elevatorLevel = 0;
            currentTarget = levels[0];
        }
    }
}