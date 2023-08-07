package frc.robot.nerdyfiles.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.nerdyfiles.swerve.configuration.ModuleConfiguration;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class FXSwerveModule {

  /**
   * The number of ticks in a single revolution for the encoder being used
   * https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
   */
  private static final double kAngleEncoderTicksPerRotation = 4096;
  private static final double kDriveEncoderTicksPerRotation = 2048;

  private final int moduleNumber;
  private final TalonFX driveMotor;
  private final TalonFX angleMotor;
  private final CANCoder canCoder;

  private final double kAngleSensorPositionCoefficient;
  private final double kDriveSensorPositionCoefficient;
  private final double kDriveSensorVelocityCoefficient;

  public FXSwerveModule(int moduleNumber, int driveMotorPort, int angleMotorPort, int angleMotorEncoderPort, Rotation2d angleMotorOffset, ModuleConfiguration moduleConfiguration) {
    this.moduleNumber = moduleNumber;

    kDriveSensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / kDriveEncoderTicksPerRotation;
    kDriveSensorVelocityCoefficient = kDriveSensorPositionCoefficient * 10.0;
    kAngleSensorPositionCoefficient = 360 / kAngleEncoderTicksPerRotation;

    // Angle Motor Encoder
    canCoder = new CANCoder(angleMotorEncoderPort);
    configureCANcoder(canCoder, angleMotorOffset, !moduleConfiguration.isSteerInverted());

    // Angle Motor
    angleMotor = new TalonFX(angleMotorPort);
    configureAngleMotor(angleMotor, canCoder.getDeviceID());

    angleMotor.setSensorPhase(true);
    angleMotor.setInverted(moduleConfiguration.isSteerInverted() ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
    //angleMotor.setNeutralMode(NeutralMode.Brake);

    // Drive Motor
    driveMotor = new TalonFX(driveMotorPort);
    configureDriveMotor(driveMotor);

    driveMotor.setSensorPhase(true);
    driveMotor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setSelectedSensorPosition(0);

    // Reduce CAN status frame rates
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250, 250);
    angleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250, 250);
  }

  private static ErrorCode configureCANcoder(CANCoder canCoder, Rotation2d angleMotorOffset, boolean sensorDirection) {
    canCoder.configFactoryDefault();

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.magnetOffsetDegrees = angleMotorOffset.getDegrees();

    return canCoder.configAllSettings(config, 250);
  }

  private static ErrorCode configureDriveMotor(TalonFX motor) {
    motor.configFactoryDefault();

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.statorCurrLimit = CTREUtils.defaultCurrentLimit();

    return motor.configAllSettings(configuration, 250);
  }

  private static ErrorCode configureAngleMotor(TalonFX motor, int canCoderDeviceID) {
    motor.configFactoryDefault();

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.statorCurrLimit = CTREUtils.defaultCurrentLimit();

    configuration.slot0.kP = 0.9; //0.3
    configuration.slot0.kI = 0.0;
    configuration.slot0.kD = 0.1;

    // Use the CANCoder as the remote sensor for the primary TalonFX PID
    configuration.remoteFilter0.remoteSensorDeviceID = canCoderDeviceID;
    configuration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

    return motor.configAllSettings(configuration, 250);
  }

  /**
   * Gets the rotational position of the module. Should be between
   * 0 and 360 degrees.
   * @return The rotational position of the angle motor in degrees
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
  }

  /**
   * Get the velocity of the drive motor for the module.
   * @return The velocity for the drive motor of the module in meters per second.
   */
  private double getVelocity() {
    return driveMotor.getSelectedSensorVelocity() * kDriveSensorVelocityCoefficient;
  }

  public void set(SwerveModuleState desiredState, double maxSpeedMetersPerSecond) {
    Rotation2d currentAngle = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentAngle);

    // Find the rotational difference between the current state and the desired state
    // Will be a positive value for clockwise rotations, neg for ccw rotations
    Rotation2d rotationDelta = state.angle.minus(currentAngle);

    double deltaTicks = (rotationDelta.getDegrees() / 360) * kAngleEncoderTicksPerRotation;
    double currentTicks = canCoder.getPosition() / kAngleSensorPositionCoefficient;
    double desiredTicks = currentTicks + deltaTicks;

    // Set the position for the motor by converting our degrees to # of ticks for the rotational value
    angleMotor.set(TalonFXControlMode.Position, desiredTicks);

    driveMotor.set(
      TalonFXControlMode.PercentOutput,
      state.speedMetersPerSecond / maxSpeedMetersPerSecond
    );
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getSelectedSensorPosition() * kDriveSensorPositionCoefficient, getAngle());
  }

  /**
   * Gets the angle motor temperature
   * @return - The temperature of the angle motors
   */
  private double getAngleMotorTemperature() {
    return angleMotor.getTemperature();
  }

  /**
   * Gets the drive motor temperature
   * @return - The temperature of the drive motors
   */
  private double getDriveMotorTemperature() {
    return driveMotor.getTemperature();
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    angleMotor.setNeutralMode(neutralMode);
    driveMotor.setNeutralMode(neutralMode);
  }

  public void logDebug() {
    if (Constants.DashboardLogging.SWERVE) {
      String prefix = "Swerve/" + moduleNumber + "/";
      SmartDashboard.putNumber(prefix + "Angle Position (Degrees)", canCoder.getPosition());
      SmartDashboard.putNumber(prefix + "Angle Absolute Position (Degrees)", canCoder.getAbsolutePosition());
      SmartDashboard.putNumber(prefix + "Velocity (ft per s)", Units.metersToFeet(getVelocity()));
      SmartDashboard.putNumber(prefix + "Drive Motor Temperature (C)", getDriveMotorTemperature());
      SmartDashboard.putNumber(prefix + "Angle Motor Temperature (C)", getAngleMotorTemperature());
      SmartDashboard.putNumber(prefix + "Angle Motor Error", angleMotor.getClosedLoopError());
    }
  }

}
