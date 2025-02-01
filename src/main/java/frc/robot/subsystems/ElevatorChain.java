package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ElevatorChain extends SubsystemBase {
    
    public SparkMax m_leadMotor;
    public SparkMax m_followMotor;
    public boolean isOn = false;
    public double goal;
    public int elevatorLevel = 0;
  
    private static final int kMotorPort = 9;
    private static final int kEncoderChannel = 4;

    private static double ElevatorKp = 5.0;
    public static final double kMinElevatorHeight = 0.0;
    public static final double kMaxElevatorHeight = Units.inchesToMeters(50);
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;

    public static final DutyCycleEncoder m_encoder = new DutyCycleEncoder(kEncoderChannel);

    public double controlLoop(double measurment, double setpoint){
    double error = setpoint - measurment;   //Error = Setpoint - Measurement
    double output = error * ElevatorKp;  //P Contorl Loop is Output = Position Error * Kp
    
  //Feed Forward
    //The Feed Forward term can help account for gravity
    //there is some amount of motor power alway needed to react against gravity
    double feedForward = 0.1;
    output = output + feedForward;

    //Tolerance
    //If we are with in the tolerance output is zero
    double tolerance = 0.5; //Tolerance for control loop in inches
    if (Math.abs(error) < Units.inchesToMeters(tolerance)){
      output = 0;
    }

    //Minimum Output
    double sign = Math.signum(output);
    double minOutput = 0.05;
    if (Math.abs(output) < minOutput){
      output = minOutput * sign;
    }

    //Min and max values
    //If we are at our min value don't let us drive down, if we are at max don't let us drive up
    if (output < (0 + feedForward) && measurment < kMinElevatorHeight){
      output = 0;
    }
    if (output > 0 && measurment > kMaxElevatorHeight ){
      output = 0;
    }
    
    return output;
    }
  
    public double getDistance(){
     return m_encoder.get() * kElevatorEncoderDistPerPulse;
    }

    public void init(){
    
    m_leadMotor = new SparkMax(CANConstants.armsMain, MotorType.kBrushless);
    SparkMaxConfig m_leadMotor_config = new SparkMaxConfig();
    m_leadMotor_config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    m_leadMotor.configure(m_leadMotor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_followMotor = new SparkMax(CANConstants.armsFollow, MotorType.kBrushless);
    SparkMaxConfig m_followMotor_config = new SparkMaxConfig();
    m_followMotor_config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    m_leadMotor.configure(m_followMotor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Lebel", elevatorLevel);
    if(isOn){
      m_leadMotor.setVoltage(controlLoop(this.getDistance(), Units.inchesToMeters(goal)));
      m_followMotor.setVoltage(-1 * controlLoop(this.getDistance(), Units.inchesToMeters(goal)));
    }
  }

  public void start(double position){
    elevatorLevel = 1; // placeholder for now
    goal = position; // in meters
    isOn = true;
  }

  public void stop(){

    m_leadMotor.setVoltage(controlLoop(this.getDistance(), Units.inchesToMeters(0)));
    m_followMotor.setVoltage(-1 * controlLoop(this.getDistance(), Units.inchesToMeters(0)));

    isOn=false;
  }
}
