package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericSubsystem;

public class GenericCommand extends Command {
    GenericSubsystem m_subsystem;
    double setSpeed;
    
    public GenericCommand(GenericSubsystem subsystem, double speed) {
        m_subsystem = subsystem;
        setSpeed = speed;
        addRequirements(m_subsystem);
    }   
    
    @Override
    public void initialize() {
        if (setSpeed != 0) {
            m_subsystem.start(setSpeed);
        } else {
            m_subsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }
}
