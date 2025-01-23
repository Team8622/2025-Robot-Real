package frc.robot.commands.auton.Full;

import java.io.Console;
import java.sql.DriverPropertyInfo;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.support.AutoBalanceCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.Intake_Yay;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Drive;

public class DriveForward2 extends Command {
    public DriveSubsystem drive;
    //public Intake_Yay intake;
    Timer timer = new Timer();
    /*public DriveForward2(DriveSubsystem drive, Intake_Yay intake){
        addRequirements(RobotContainer.m_driveTrain);
        addRequirements(RobotContainer.m_intake);
        this.drive = drive;
        this.intake =intake;
        // then my code
        
    }*/
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute(){
        if(timer.get() > .5){
        drive.drive(-3.0, 0, 0.0, true);
        //intake.start(IntakeConstants.inSpeed);

        System.out.println(timer.get());
        }

        if(timer.get() > 3.5) {
            drive.drive(0,0,0,true);
        }
    }
    @Override
    public boolean isFinished() {
        if (timer.get() > 3.5){
          System.out.println("IS FINISHED");
          return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        drive.drive(0, 0, 0, true);
        //intake.stop();
        System.out.println("done woop woop");
    }
}