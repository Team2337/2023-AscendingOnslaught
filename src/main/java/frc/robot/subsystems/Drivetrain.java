package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriverDashboardPositions;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.nerdyfiles.swerve.FXSwerveModule;

public class Drivetrain extends SubsystemBase {

  private final PigeonIMU pigeon;
  private Accelerometer accelerometer = new BuiltInAccelerometer();

  /**
   * Array for swerve module objects, sorted by ID
   * 0 is Front Right,
   * 1 is Front Left,
   * 2 is Back Left,
   * 3 is Back Right
   */
  private final FXSwerveModule[] modules;

  /**
   * Should be in the same order as the swerve modules (see above)
   * Positive x values represent moving toward the front of the robot
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES)
    )
  );

  //private final SwerveDrivePoseEstimator odometry;
  private final SwerveDrivePoseEstimator odometry;
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {null, null, null, null};

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private ChassisSpeeds realChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // Update Drivetrain state only once per cycle
  private Pose2d pose = new Pose2d();
  // private Field2d field = new Field2d();

  // Array for Yaw Pitch and Roll values in degrees
  private double[] ypr_deg = { 0, 0, 0 };

  /**
   * Subsystem where swerve modules are configured,
   * and the calculations from the joystick inputs is handled.
   * Field orientation is set here as well
   */
  public Drivetrain(PigeonIMU pigeon) {
    this.pigeon = pigeon;

    modules = new FXSwerveModule[] {
      new FXSwerveModule(
        Constants.Swerve.ModulePosition.FRONT_RIGHT.value,
        Constants.getInstance().MODULE0_DRIVE_MOTOR_ID,
        Constants.getInstance().MODULE0_ANGLE_MOTOR_ID,
        Constants.getInstance().MODULE0_ANGLE_CANCODER_ID,
        Constants.getInstance().MODULE0_ANGLE_OFFSET,
        Constants.getInstance().SWERVE_MODULE_CONFIGURATION
      ),
      new FXSwerveModule(
        Constants.Swerve.ModulePosition.FRONT_LEFT.value,
        Constants.getInstance().MODULE1_DRIVE_MOTOR_ID,
        Constants.getInstance().MODULE1_ANGLE_MOTOR_ID,
        Constants.getInstance().MODULE1_ANGLE_CANCODER_ID,
        Constants.getInstance().MODULE1_ANGLE_OFFSET,
        Constants.getInstance().SWERVE_MODULE_CONFIGURATION
      ),
      new FXSwerveModule(
        Constants.Swerve.ModulePosition.BACK_LEFT.value,
        Constants.getInstance().MODULE2_DRIVE_MOTOR_ID,
        Constants.getInstance().MODULE2_ANGLE_MOTOR_ID,
        Constants.getInstance().MODULE2_ANGLE_CANCODER_ID,
        Constants.getInstance().MODULE2_ANGLE_OFFSET,
        Constants.getInstance().SWERVE_MODULE_CONFIGURATION
      ),
      new FXSwerveModule(
        Constants.Swerve.ModulePosition.BACK_RIGHT.value,
        Constants.getInstance().MODULE3_DRIVE_MOTOR_ID,
        Constants.getInstance().MODULE3_ANGLE_MOTOR_ID,
        Constants.getInstance().MODULE3_ANGLE_CANCODER_ID,
        Constants.getInstance().MODULE3_ANGLE_OFFSET,
        Constants.getInstance().SWERVE_MODULE_CONFIGURATION
      ),
    };

    
    modulePositions[0] = modules[0].getPosition();
    modulePositions[1] = modules[1].getPosition();
    modulePositions[2] = modules[2].getPosition();
    modulePositions[3] = modules[3].getPosition();

    odometry = new SwerveDrivePoseEstimator(
      kinematics, 
      getGyroscopeRotation(), 
      modulePositions,
      pose);
    setupShuffleboard(Constants.DashboardLogging.DRIVETRAIN);
    // SmartDashboard.putData("field", field);
  }

  private void setupShuffleboard(Boolean logEnable) {
    // Normal debug
    if (logEnable) {
      ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
      ShuffleboardLayout chassisSpeedsWidget = tab.getLayout("Chassis Speeds", BuiltInLayouts.kList)
        .withSize(4, 8)
        .withPosition(12, 0);
      chassisSpeedsWidget.addNumber("vx meters per s", () -> chassisSpeeds.vxMetersPerSecond);
      chassisSpeedsWidget.addNumber("vy meters per s", () -> chassisSpeeds.vyMetersPerSecond);
      chassisSpeedsWidget.addNumber("omega radians per s", () -> chassisSpeeds.omegaRadiansPerSecond);

      ShuffleboardLayout gyroWidget = tab.getLayout("Gyro", BuiltInLayouts.kList)
      .withSize(4, 8)
      .withPosition(16, 0);
      gyroWidget.addNumber("Degrees", () -> getGyroscopeRotation().getDegrees());
    }

    if (Constants.DO_SYSTEMS_CHECK) {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;

      systemsCheck.addNumber("Roll", () -> getGyroscopeRoll().getDegrees())
        .withPosition(SystemsCheckPositions.ROLL_DEGREES.x, SystemsCheckPositions.ROLL_DEGREES.y)
        .withSize(3, 3);
    }
    
    // Driver Dashboard
    Constants.DRIVER_DASHBOARD.addNumber("Gyro Degrees", () -> getGyroscopeRotation().getDegrees())
      .withPosition(DriverDashboardPositions.GYRO_DEGREES.x, DriverDashboardPositions.GYRO_DEGREES.y)
      .withSize(DriverDashboardPositions.GYRO_DEGREES.width, DriverDashboardPositions.GYRO_DEGREES.height);
  }

  // public Field2d getField2d() {
  //   return field;
  // }
  
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    odometry.addVisionMeasurement(visionPose, timestampSeconds);
  }

  public void resetPosition(Pose2d pose) {
    odometry.resetPosition(getGyroscopeRotation(), modulePositions, pose);
    pose = odometry.getEstimatedPosition();
  }

  public Pose2d getPose() {
    return pose;
  }

  public Translation2d getTranslation() {
    return getPose().getTranslation();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Get the gyroscope rotation of the robot as measured by the
   * yaw value of the Pigeon. [-368,640, 368,640] degrees.
   * Counter-clockwise is interpreted as a positive change,
   * clockwise is interpreted as a negative change.
   *
   * @return The rotation of the robot.
   */
  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(ypr_deg[0]);
  }

  public Rotation2d getGyroscopePitch() {
    return Rotation2d.fromDegrees(ypr_deg[1]);
  }
  public Rotation2d getGyroscopeRoll() {
    return Rotation2d.fromDegrees(ypr_deg[2]);
  }

  public PigeonState getPigeonState() {
    return pigeon.getState();
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  /**
   * Get the combined x + y velocity vector for the robot.
   */
  public double velocity() {
    return Math.hypot(
      realChassisSpeeds.vxMetersPerSecond,
      realChassisSpeeds.vyMetersPerSecond
    );
  }

  public boolean isMoving() {
    return !Utilities.withinTolerance(0, velocity(), 0.01); //0.001
  }

  /**
   * Stops all of the motors on each module
   */
  public void stopMotors() {
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    pigeon.getYawPitchRoll(ypr_deg);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    modulePositions[0] = modules[0].getPosition();
    modulePositions[1] = modules[1].getPosition();
    modulePositions[2] = modules[2].getPosition();
    modulePositions[3] = modules[3].getPosition();

    for (int i = 0; i < states.length; i++) {
      FXSwerveModule module = modules[i];
      SwerveModuleState moduleState = states[i];

      module.set(moduleState, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);
      module.logDebug();
    }

    SwerveModuleState[] realStates = {
      modules[0].getState(),
      modules[1].getState(),
      modules[2].getState(),
      modules[3].getState()
    };

    realChassisSpeeds = kinematics.toChassisSpeeds(realStates);

    pose = odometry.update(
      getGyroscopeRotation(),
      modulePositions
    );

    // field.setRobotPose(pose);
    /*
    Logger.getInstance().recordOutput("Odometry/Robot",
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
    */
    SmartDashboard.putNumber("Pose X", Units.metersToInches(pose.getX()));
    SmartDashboard.putNumber("Pose Y", Units.metersToInches(pose.getY()));
    SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Gyro", getGyroscopeRotation().getDegrees());
    SmartDashboard.putNumber("Roll", getGyroscopeRoll().getDegrees());
    SmartDashboard.putNumber("Pitch", getGyroscopePitch().getDegrees());
    SmartDashboard.putNumber("Accelerometer X", accelerometer.getX());
    SmartDashboard.putNumber("Accelerometer Y", accelerometer.getY());
    SmartDashboard.putNumber("Accelerometer Z", accelerometer.getZ());
    SmartDashboard.putNumber("Module 0", Units.metersToInches(modules[0].getPosition().distanceMeters));
    SmartDashboard.putNumber("Module 1", Units.metersToInches(modules[1].getPosition().distanceMeters));
    SmartDashboard.putNumber("Module 2", Units.metersToInches(modules[2].getPosition().distanceMeters));
    SmartDashboard.putNumber("Module 3", Units.metersToInches(modules[3].getPosition().distanceMeters));

  }

}