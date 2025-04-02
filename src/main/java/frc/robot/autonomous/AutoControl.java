// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoControl extends Command {
  /** Creates a new ManualControl. */
  Elevator m_elevator;
  double setSpeed;
  double setTime;
  private final Timer timer = new Timer();

  public AutoControl(Elevator subsystem, double speed, double time) {
    m_elevator = subsystem;
    setSpeed = speed;
    setTime = time;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("Init: " + setSpeed);
    m_elevator.setManualPower(setSpeed);
  }
  @Override
  public boolean isFinished() {
    return timer.get() >= setTime; // Stop after 1 second
}
  @Override
  public void end(boolean interrupted){
      System.out.println("End: " + setSpeed);
      m_elevator.setManualPower(0);
  }
}
