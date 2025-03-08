// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualControl extends Command {
  /** Creates a new ManualControl. */
  Elevator m_elevator;
  double speed;
  public ManualControl(Elevator subsystem, double power) {
    m_elevator = subsystem;
    speed = power;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setManualPower(speed);
  }

  @Override
  public void end(boolean interrupted){
      m_elevator.setManualPower(0);
  }
}
