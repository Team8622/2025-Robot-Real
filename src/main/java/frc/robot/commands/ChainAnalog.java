package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ChainAnalog extends Command{
    Elevator m_chain;
    double setLevel;

    public ChainAnalog(Elevator chain, double level){
        m_chain = chain;
        setLevel = level;
        addRequirements(m_chain);
    }   
    
    @Override
    public void initialize(){
        m_chain.setLevel(setLevel);
    }
}

