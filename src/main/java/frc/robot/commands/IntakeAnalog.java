package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral_Intake;

public class IntakeAnalog extends Command{
    Coral_Intake m_intake;
    double setSpeed;
    
    public IntakeAnalog(Coral_Intake intake, double speed){
        m_intake = intake;
        setSpeed = speed;
        addRequirements(m_intake);
    }   
    
    @Override
    public void initialize(){
        m_intake.start(setSpeed);

    }

    @Override
    public void end(boolean interrupted){
        m_intake.stop();
    }
}
