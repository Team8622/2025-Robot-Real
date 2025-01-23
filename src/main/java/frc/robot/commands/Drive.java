package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class Drive extends Command{
    public Drive(DriveSubsystem subsystem){
        
        addRequirements(RobotContainer.m_driveTrain);
    }

    SlewRateLimiter filter = new SlewRateLimiter(1); 
    
    @Override
    public void execute(){
        RobotContainer.m_driveTrain.drive(
            RobotContainer.driver.getLeftY()*1 /* xAxis  */* Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            RobotContainer.driver.getLeftX()*-1/* yAxis */ *Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            RobotContainer.driver.getRightX() /* rot CCW positive */ * Constants.DriveConstants.kMaxRotationalSpeed, 
            true
        );
    }
}
