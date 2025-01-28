package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae_Intake;


public class AlgaeAnalog extends Command{
    Algae_Intake m_intake;
    double setSpeed;
    
    public AlgaeAnalog(Algae_Intake intake, double speed){
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
