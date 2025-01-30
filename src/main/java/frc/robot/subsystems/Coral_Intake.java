// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Here's the general line of thought for operation:
 * * A decided starting position is agreed upon (maybe completely closed, maybe somewhere else), encoder val is reset (0)
 * * Manny opens to its "open position" setpoint (some value of rotations), encoder val is reset (0)
 * * When it is closed on a cone or cube, it closes some amount of rotations, encoder val is reset (0)
 * * To get back to open position, manny reverses that number of rotations, encoder val is reset (0), etc.
 */

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

//import edu.wpi.fir077655 t.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class Coral_Intake extends SubsystemBase{
  
  public SparkMax m_lead;
  public SparkMax m_feeder;
  
  //The manipulator starts being as closed as possible (starting position)
  //for closePos: 0 = no piece (init), 1 = cone, 2 = cube
  public boolean isOn = false;
  //public int closePos = 0;

  /*
  private double kp = MannyConstants.kPManny;
  private double ki = MannyConstants.kIManny;
  private double kd = MannyConstants.kDManny;
  */

  /*
  *Constructor
  *defines a PID Controller with the subsystem, muy importante
  */
  public Coral_Intake() {
  }

  public void init(){
    //initialize all the things
    m_lead = new SparkMax(CANConstants.intakeMain, MotorType.kBrushless);
    SparkMaxConfig m_lead_config = new SparkMaxConfig();
    m_lead_config
    .inverted(true)
    .idleMode(IdleMode.kCoast);
   // m_lead.setInverted(true);
   // m_lead.restoreFactoryDefaults();
    m_lead.configure(m_lead_config, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    m_feeder = new SparkMax(CANConstants.intakeSushi, MotorType.kBrushless);
    SparkMaxConfig m_feeder_config = new SparkMaxConfig();
    m_feeder_config
    .inverted(true)
    .idleMode(IdleMode.kCoast);
    m_feeder.configure(m_feeder_config, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

   // old configuration
   // m_feeder.restoreFactoryDefaults();
   // m_feeder.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Intake", isOn);
  }

  public void start(double speed){
    m_lead.set(speed);
    m_feeder.set(-speed);
    
    isOn = true;
  }

  public void stop(){
    m_lead.set(0);
    m_feeder.set(0);
    isOn = false;
  }

  public Command startCommand(double speed){
    return this.run(()->start(speed));
  }

  public Command stopCommand(){
    return this.run(()->stop());
  }
}
