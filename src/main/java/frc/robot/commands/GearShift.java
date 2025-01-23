package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;

public class GearShift extends InstantCommand{
    
    public GearShift(){
    }

    @Override
    public void initialize(){
        if(DriveConstants.kMaxSpeedMetersPerSecond == 8.0){
            DriveConstants.kMaxSpeedMetersPerSecond = 4;
        }else{
            DriveConstants.kMaxSpeedMetersPerSecond = 8;
        }
    }
}
