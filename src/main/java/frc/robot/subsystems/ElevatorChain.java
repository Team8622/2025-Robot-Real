package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ElevatorChain extends SubsystemBase {
    
    public SparkMax m_leadMotor;
    public SparkMax m_followMotor;
    
    public boolean isOn = false;


    
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
    SmartDashboard.putBoolean("Elevator Up", isOn);
  }

  public void start(double speed){
    
    m_leadMotor.set(speed);
    m_followMotor.set(-speed);
    
    isOn = true;
  }

  public void stop(){
 

    m_leadMotor.set(0);
    m_followMotor.set(0);

    isOn=false;
  }
}
