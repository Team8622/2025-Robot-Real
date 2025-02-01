package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorChain;

public class ChainAnalog extends Command{
    ElevatorChain m_chain;
    double setGoal;
    
    public ChainAnalog(ElevatorChain chain, double goal){
        m_chain = chain;
        setGoal = goal;
        addRequirements(m_chain);
    }   
    
    @Override
    public void initialize(){
        m_chain.start(setGoal);
    }

    @Override
    public void end(boolean interrupted){
        m_chain.stop();
    }
}

