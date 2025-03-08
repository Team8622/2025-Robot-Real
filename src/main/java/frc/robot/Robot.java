// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Drive;
//import frc.robot.subsystems.ElevatorSimSubsystem;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private RobotContainer m_robotContainer;
  //Thread m_visionThread;


  // distance per pulse = (distance per revolution) / (pulses per revolution)
  // = (Pi * D) / ppr

  // Standard classes for controlling our elevator

  @Override
  public void robotInit() {
    //AddressableLED m_led = new AddressableLED(9);
    // AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    //m_led.setLength(m_ledBuffer.getLength());

    // for(var i = 0;i< m_ledBuffer.getLength();i++){
    //   m_ledBuffer.setRGB(i, 255, 0, 0);
    // }
    //m_led.setData(m_ledBuffer);
    //m_led.start();


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
   
    m_robotContainer.init();
    //m_robotContainer.m_driveTrain.zeroHeading();

    

    //camera code
  //   m_visionThread =
  //   new Thread(
  //       () -> {
  //         UsbCamera camera = CameraServer.startAutomaticCapture();
  //         camera.setResolution(640, 480);
  //         CvSink cvSink = CameraServer.getVideo();
  //         CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
  //         Mat mat = new Mat();
  //         while (!Thread.interrupted()) {
  //           if (cvSink.grabFrame(mat) == 0) {
  //             outputStream.notifyError(cvSink.getError());
  //             continue;
  //           }
  //           Imgproc.rectangle(
  //               mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);

  //           outputStream.putFrame(mat);
  //         }
  //       });
  // m_visionThread.setDaemon(true);
  // m_visionThread.start();
  //end camera code
    
 
    //m_chooser.addOption("Deposit GP and move", kDepositMobility);
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    /* /* 
    if(m_robotContainer.m_PhotonCam.photonCamera.getLatestResult().hasTargets()){
    SmartDashboard.putNumber("Yaw I guess", m_robotContainer.m_PhotonCam.getVisionResultYaw().getX());
    }*/
    //RobotContainer.m_driveTrain.periodic();
    CommandScheduler.getInstance().run();
    MotorSafety.checkMotors();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.m_intake.stop();

  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Command m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if( m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }
  /** This function is called periodicallya during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  
  @Override
  public void teleopInit() {
    Command m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
       m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().run();
    RobotContainer.m_driveTrain.setDefaultCommand(
       new Drive(RobotContainer.m_driveTrain)
    );
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }         

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
