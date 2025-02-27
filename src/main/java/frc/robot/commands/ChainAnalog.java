package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ChainAnalog extends Command{
    Elevator m_chain;
    int levelDelta;

    public ChainAnalog(Elevator chain, int delta){
        m_chain = chain;
        levelDelta = delta;
        addRequirements(m_chain);
    }   
    
    @Override
    public void initialize(){
        m_chain.adjustLevel(levelDelta);
    }
}

