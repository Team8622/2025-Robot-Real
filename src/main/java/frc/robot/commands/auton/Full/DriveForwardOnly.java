package frc.robot.commands.auton.Full;

import java.io.Console;
import java.sql.DriverPropertyInfo;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.support.AutoBalanceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Drive;

public class DriveForwardOnly extends Command {
    public DriveSubsystem drive;
    Timer timer = new Timer();
    public DriveForwardOnly(DriveSubsystem drive){
        addRequirements(RobotContainer.m_driveTrain);
        this.drive = drive;
        // then my code
        
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute(){
        if(timer.get() > .5){
        drive.drive(-3.0, 0, 0.0, true);

        System.out.println(timer.get());
        }

    }
    @Override
    public boolean isFinished() {
        if (timer.get() > 2){
          System.out.println("IS FINISHED");
          return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        drive.drive(0, 0, 0, true);
        System.out.println("done woop woop");
    }
}