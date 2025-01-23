


 package frc.robot.commands.auton.Full;

import java.sql.DriverPropertyInfo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.support.AutoBalanceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive;

public class BalanceFullCommand extends Command {
    public DriveSubsystem subsystem;
    public BalanceFullCommand(DriveSubsystem subsystem){
        //addCommands(new RunCommand(() ->RobotContainer.m_driveTrain.drive(0.0, 0.5 , 0.0, true)), new BalanceFullCommand(subsystem));
    }

    @Override
    public void initialize(){
        //addCommands(new RunCommand(() -> )
    }
}         