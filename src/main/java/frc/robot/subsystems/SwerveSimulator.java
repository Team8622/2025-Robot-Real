// package frc.robot.subsystems;

// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructArrayPublisher;

// public class SwerveSimulator extends GenericSubsystem {
//   private final StructArrayPublisher<SwerveModuleState> publisher;
//   private final SwerveSubsystem driveSubsystem;

//   public SwerveSimulator(SwerveSubsystem driveSubsystem) {
//     this.driveSubsystem = driveSubsystem;
//     // Start publishing an array of module states with the "/SwerveStates" key
//     publisher = NetworkTableInstance.getDefault()
//       .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
//   }

//   @Override
//   public void periodic() {
//     // Periodically send a set of module states
//     // TODO finish this code
//     //publisher.set(driveSubsystem.getModuleStates());
//   }
// }