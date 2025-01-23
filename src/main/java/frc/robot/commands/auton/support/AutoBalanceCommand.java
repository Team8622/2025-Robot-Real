package frc.robot.commands.auton.support;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends Command{
    public DriveSubsystem subsystem;
    public AutoBalanceCommand(DriveSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(RobotContainer.m_driveTrain);
        
    }
    
    public PIDController greg = new PIDController(.08, 0, 0);
    
    
    @Override
    public void execute(){
        subsystem.drive(
            greg.calculate(subsystem.getPitch(), 0),
            0,
            0, 
            true
        );
        System.out.println(subsystem.getPitch());
    }
}
