package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends GenericSubsystem {
	// The gyro sensor
	// public final Gyro m_gyro = new AHRS(SPI.Port.kMXP);
	public final AHRS m_ahrs = new AHRS(AHRS.NavXComType.kMXP_SPI);

	private int gyroOffset = 0;
	// Locations for the swerve drive modules relative to the robot center.
	Translation2d m_frontLeftLocation = new Translation2d(0.276, 0.282);
	Translation2d m_frontRightLocation = new Translation2d(0.280, -0.275);
	Translation2d m_backLeftLocation = new Translation2d(-0.284, 0.280);
	Translation2d m_backRightLocation = new Translation2d(-0.285, -0.280);
	private ChassisSpeeds speeds = new ChassisSpeeds();
	private boolean drivingRobotRelative = false;
	// Creating my kinematics object using the module locations
	SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	// Odometry class for tracking robot pose
	// public SwerveDrivePoseEstimator m_odometry = new
	// SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,m_ahrs.getRotation2d(),new
	// SwerveModulePosition[]{
	// new SwerveModulePosition(),new SwerveModulePosition(), new
	// SwerveModulePosition(), new SwerveModulePosition()},
	// new Pose2d());

	private final ShuffleboardTab moduleTab = Shuffleboard.getTab("Module Info");
	private final SwerveModule m_frontLeft = new SwerveModule(
			DriveConstants.kFrontLeftDriveMotorPort,
			DriveConstants.kFrontLeftTurningMotorPort,
			DriveConstants.kFrontLeftTurningEncoderPort,
			DriveConstants.kFrontLeftAngleZero,
			DriveConstants.kFrontLeftTurningEncoderReversed,
			DriveConstants.kFrontLeftDriveEncoderReversed,
			moduleTab.getLayout("Front Left Module", BuiltInLayouts.kList)
					.withSize(4, 8)
					.withPosition(0, 0));
	private final SwerveModule m_rearLeft = new SwerveModule(
			DriveConstants.kRearLeftDriveMotorPort,
			DriveConstants.kRearLeftTurningMotorPort,
			DriveConstants.kRearLeftTurningEncoderPort,
			DriveConstants.kRearLeftAngleZero,
			DriveConstants.kRearLeftTurningEncoderReversed,
			DriveConstants.kRearLeftDriveEncoderReversed,
			moduleTab.getLayout("Rear Left Module", BuiltInLayouts.kList)
					.withSize(4, 8)
					.withPosition(4, 0));
	private final SwerveModule m_frontRight = new SwerveModule(
			DriveConstants.kFrontRightDriveMotorPort,
			DriveConstants.kFrontRightTurningMotorPort,
			DriveConstants.kFrontRightTurningEncoderPort,
			DriveConstants.kFrontRightAngleZero,
			DriveConstants.kFrontRightTurningEncoderReversed,
			DriveConstants.kFrontRightDriveEncoderReversed,
			moduleTab.getLayout("Front Right Module", BuiltInLayouts.kList)
					.withSize(4, 8)
					.withPosition(8, 0));
	private final SwerveModule m_rearRight = new SwerveModule(
			DriveConstants.kRearRightDriveMotorPort,
			DriveConstants.kRearRightTurningMotorPort,
			DriveConstants.kRearRightTurningEncoderPort,
			DriveConstants.kRearRightAngleZero,
			DriveConstants.kRearRightTurningEncoderReversed,
			DriveConstants.kRearRightDriveEncoderReversed,
			moduleTab.getLayout("Rear Right Module", BuiltInLayouts.kList)
					.withSize(4, 8)
					.withPosition(12, 0));

	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
			m_kinematics, m_ahrs.getRotation2d(),
			new SwerveModulePosition[] {
					m_frontLeft.getPosition(),
					m_frontRight.getPosition(),
					m_rearLeft.getPosition(),
					m_rearRight.getPosition()
			}, new Pose2d());

	/**
	 * Creates a new DriveSubsystem.
	 */
	public DriveSubsystem() {

		// AutoBuilder.configureHolonomic(
		// this::getPose, // Robot pose supplier
		// this::resetPose, // Method to reset odometry (will be called if your auto has
		// a starting pose)
		// this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
		// RELATIVE
		// this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE
		// ChassisSpeeds
		// new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
		// likely live in your Constants class
		// new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
		// new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
		// 4.5, // Max module speed, in m/s
		// 0.4, // Drive base radius in meters. Distance from robot center to furthest
		// module.
		// new ReplanningConfig() // Default path replanning config. See the API for the
		// options here
		// ),
		// () -> {
		// // Boolean supplier that controls when the path will be mirrored for the red
		// alliance
		// // This will flip the path being followed to the red side of the field.
		// // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

		// var alliance = DriverStation.getAlliance();
		// if (alliance.isPresent()) {
		// return alliance.get() == DriverStation.Alliance.Red;
		// }
		// return false;
		// },
		// this // Reference to this subsystem to set requirements
		// );

		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
			// Todo: fix the method that sets chassispeeds. runvelocity is incorrect but i need to better understand the swerve code to fix it.
			// Configure AutoBuilder last
			AutoBuilder.configure(
					this::getPose, // Robot pose supplier
					this::resetPose, // Method to reset odometry (will be called if your auto has a
										// starting pose)
					this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
					(speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the
																	// robot given ROBOT RELATIVE
																	// ChassisSpeeds. Also optionally
																	// outputs individual module
																	// feedforwards
					new PPHolonomicDriveController( // PPHolonomicController is the built in path
													// following controller for holonomic drive
													// trains
							new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
							new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
					),
					config, // The robot configuration
					() -> {
						// Boolean supplier that controls when the path will be mirrored for the
						// red alliance
						// This will flip the path being followed to the red side of the field.
						// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

						var alliance = DriverStation.getAlliance();
						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}
						return false;
					},
					this // Reference to this subsystem to set requirements
			);
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}
	}

	public void runVelocity(ChassisSpeeds speeds) {
		// Calculate module setpoints
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
		SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, 5.5); // DOUBLE CHECK THIS VALUE

		// Send setpoints to modules
		SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

	}

	public double speed() {
		return 0;
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(
				m_ahrs.getRotation2d(),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_rearLeft.getPosition(),
						m_frontRight.getPosition(),
						m_rearRight.getPosition()
				});

		/*
		 * SmartDashboard.putString("m_frontLeft", m_frontLeft.getState().toString());
		 * SmartDashboard.putString("m_rearLeft", m_rearLeft.getState().toString());
		 * SmartDashboard.putString("m_frontRight", m_frontRight.getState().toString());
		 * SmartDashboard.putString("m_rearRight", m_rearRight.getState().toString());
		 * 
		 * // SmartDashboard.putNumber("m_frontLeftAngle",
		 * m_frontLeft.getModuleHeading());
		 * //SmartDashboard.putNumber("m_rearLeftAngle", m_rearLeft.getModuleHeading());
		 * // SmartDashboard.putNumber("m_frontRightAngle",
		 * m_frontRight.getModuleHeading());
		 * //SmartDashboard.putNumber("m_rearRightAngle",
		 * m_rearRight.getModuleHeading());
		 * 
		 * // SmartDashboard.putString("odometry",
		 * m_odometry.getPoseMeters().toString());
		 * // SmartDashboard.putString("rotation2d", m_gyro.getRotation2d().toString());
		 * SmartDashboard.putNumber("pitch", m_ahrs.getPitch());
		 * SmartDashboard.putNumber("angle", m_gyro.getAngle());
		 * m_frontLeft.periodic_func();
		 * m_rearRight.periodic_func();
		 * m_rearLeft.periodic_func();
		 * m_frontRight.periodic_func();
		 */
	}

	public void setGyroOffset(int gyroOffset) {
		this.gyroOffset = gyroOffset;
	}

	public double getPitch() {
		return (double) m_ahrs.getRoll();
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		// return m_odometry.getPoseMeters();
		return new Pose2d();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetPose(Pose2d pose) {
		pose = m_odometry.update(m_ahrs.getRotation2d(),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(), m_frontRight.getPosition(),
						m_rearLeft.getPosition(), m_rearRight.getPosition()
				});
	};

	public ChassisSpeeds getRobotRelativeSpeeds() {

		return m_kinematics.toChassisSpeeds(getModuleStates());
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		states[0] = m_frontLeft.getState();
		states[1] = m_frontRight.getState();
		states[2] = m_rearLeft.getState();
		states[3] = m_rearRight.getState();
		return states;
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */

	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		SmartDashboard.putBoolean("Field Relative:", fieldRelative);
		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
								Rotation2d.fromDegrees(-getHeading()))
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(
				swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);

		// SmartDashboard.putString("m_frontLeftDA", swerveModuleStates[0].toString());

		// SmartDashboard.putString("Front Right desired state: ",
		// /*Integer.parseInt(*/swerveModuleStates[1].toString()/*.substring(5).replaceAll("[^0-9]",
		// ""))*/);
		// SmartDashboard.putString("Front Right desired state: ",
		// swerveModuleStates[3].toString());

	}
    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)></b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
        SwerveModuleState[] m_moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(m_moduleStates);
    }

	public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        this.drivingRobotRelative = false;
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, Rotation2d.fromDegrees(-getHeading())));
    }

    /**
     * Drive the robot with the provided speeds <b>(FIELD RELATIVE)</b>
     * @param speeds
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
				desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	/**
	 * Zeroes the heading of the robot.
	 */
	public void zeroHeading() {
		gyroOffset = 0;
		m_ahrs.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		double currentHeading = m_ahrs.getRotation2d().getDegrees() + gyroOffset;
		if (currentHeading > 180) {
			currentHeading -= 360;
		} else if (currentHeading < -180) {
			currentHeading += 360;
		}
		return currentHeading;
	}
	// /**
	// * Returns the turn rate of the robot.
	// *
	// * @return The turn rate of the robot, in degrees per second
	// */
	// public double getTurnRate() {
	//// return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	// return m_gyro.getRate();
	// }

}