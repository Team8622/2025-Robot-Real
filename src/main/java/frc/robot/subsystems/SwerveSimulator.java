package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class SwerveSimulator extends GenericSubsystem {
  private final StructArrayPublisher<SwerveModuleState> publisher;
  private final DriveSubsystem driveSubsystem;

  public SwerveSimulator(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Start publishing an array of module states with the "/SwerveStates" key
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
  }

  @Override
  public void periodic() {
    // Periodically send a set of module states
    publisher.set(driveSubsystem.getModuleStates());
  }
}