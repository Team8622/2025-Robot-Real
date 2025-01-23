package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.PhotonCameraWrapper;

public class AutoAlign extends Command{
    public String POS;
    public AutoAlign(DriveSubsystem subsystem, String Stfucase){
        POS = Stfucase;
        addRequirements(RobotContainer.m_driveTrain);
    }

    //PhotonCameraWrapper cam = RobotContainer.m_PhotonCam;

    public PIDController xPos = new PIDController(Constants.AutoConstants.kAutoAlignP,0,0);
    public PIDController yPos = new PIDController(Constants.AutoConstants.kAutoAlignP,0,0);
    public PIDController yawPos = new PIDController(Constants.AutoConstants.kAutoAlignP,0,0);

    @Override
    public void execute(){
        double xSetPoint;
        switch(POS){
            case "left":
            xSetPoint = 0;
            break;
        }
        double xSpeed = xPos.calculate(0, 0);
        double ySpeed = yPos.calculate(0, 0);
        //double yawSpeed = yawPos.calculate(cam.getVisionResultYaw().getY(),0);

        RobotContainer.m_driveTrain.drive(0, 0, 0, false);
    }
}