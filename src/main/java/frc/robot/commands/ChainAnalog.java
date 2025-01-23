package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral_Intake;
import frc.robot.subsystems.ElevatorChain;

public class ChainAnalog extends Command{
    ElevatorChain m_chain;
    double setSpeed;
    
    public ChainAnalog(ElevatorChain chain, double speed){
        m_chain = chain;
        setSpeed = speed;
        addRequirements(m_chain);
    }   
    
    @Override
    public void initialize(){
        m_chain.start(setSpeed);

    }

    @Override
    public void end(boolean interrupted){
        m_chain.stop();
    }
}

