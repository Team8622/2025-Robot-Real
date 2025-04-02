// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.fasterxml.jackson.databind.deser.SettableAnyProperty;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae_Intake;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeRemove extends Command {
  /** Creates a new AlgaeRemove. */
  Algae_Intake m_algae;
  double speed;
  double setTime;
  private final Timer timer = new Timer();
  
  public AlgaeRemove(Algae_Intake intake, double inSpeed, double time) {
    m_algae = intake;
    speed = inSpeed;
    setTime = time;
    addRequirements(m_algae);
    // Use addRequirements() here to declare subsystem dependencies.
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize(){
      timer.reset();
      timer.start();
      m_algae.start(speed);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return timer.get() >= setTime; // Stop after 1 second
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
      m_algae.stop();
  }
  // Returns true when the command should end.

}