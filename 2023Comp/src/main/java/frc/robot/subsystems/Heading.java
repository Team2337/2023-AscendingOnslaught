package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;

/**
 * The Heading subsystem is a "fake" subsystem (no hardware) that is used to
 * coordinate
 * mutual exclusivity around maintaining the heading of the robot.
 * Ex: Commands that request the robot to maintain a heading should require
 * the Heading subsystem. Only one command should be updating the robot's
 * heading
 * at a time.
 */
public class Heading extends SubsystemBase {

  private static double NOMINAL_OUTPUT_STATIONARY = 0.06;
  private static double NOMINAL_OUTPUT_MOVING = 0.03;
  private static double VISION_NOMINAL_OUTPUT_STATIONARY = 0.1;

  private static double P_MOVING = 0.005;
  private static double P_STATIONARY = 0.007;
  private static double visionPValue = 0.009;
  private static double visionDValue = 0.0; // 0.001;

  private boolean visionP = false;

  /**
   * Whether or not the Heading subsystem is enabled. Being "enabled" means
   * providing a heading to maintain if a maintainHeading is set + returning
   * some value from the calculateRotation method. If the system is disabled,
   * shouldMaintainHeading will return false even when a maintainHeading is
   * set, and calculateRotation will always return 0.0
   *
   * NOTE: Currently disabled by default as not to cause confusion when
   * we're testing things on the robot.
   */
  private boolean enabled = false;

  /**
   * Supplier to provide the gyro angle of the robot. Should come from
   * the gyro on the Drivetrain. Used to calculate our maintain heading
   * calculation.
   */
  private Supplier<Rotation2d> gyroAngleSupplier;
  /**
   * Supplier to provide if the drivetrain is moving or not. This should be
   * calculated using the drivetrain's chassis speeds. This is used to set a
   * dynamic nominal value for the heading controller - higher if we're
   * stationary,
   * lower if we're moving - since it takes less power to achieve our heading
   * while we're driving then when we're stationary.
   */
  private Supplier<Boolean> drivetrainIsMovingSupplier;
  /**
   * The maintain heading is the heading we would like the robot to maintain.
   * Can be null if the robot should not maintain any specific heading.
   */
  private Rotation2d maintainHeading;
  /**
   * The next heading is the enqueued heading we would like the robot to maintain.
   * Swapping from the next heading -> the maintain heading is done via the
   * `setNextHeadingToMaintainHeading` method.
   * Next heading may be null if the robot has no next heading to maintain.
   */
  private Rotation2d nextHeading;

  /**
   * PID used to converge the robot to the maintainHeading from it's current
   * heading.
   */
  private PIDController rotationController = new PIDController(P_STATIONARY, 0.0, 0.0);

  /**
   * Heading subsystem to maintain a static heading of the robot.
   *
   * @param gyroAngleSupplier          A Supplier to provide the gyro value of the
   *                                   robot.
   *                                   Should come from the Pigeon.
   * @param drivetrainIsMovingSupplier A supplier to provide if the drivetrain is
   *                                   moving or not.
   */
  public Heading(Supplier<Rotation2d> gyroAngleSupplier, Supplier<Boolean> drivetrainIsMovingSupplier) {
    this.gyroAngleSupplier = gyroAngleSupplier;
    this.drivetrainIsMovingSupplier = drivetrainIsMovingSupplier;

    rotationController.enableContinuousInput(-180, 180);
    // +/- 1 degree position error tolerance
    rotationController.setTolerance(1.0);
  }

  @Override
  public void periodic() {
    double pCurrent = rotationController.getP();
    double pDesired = drivetrainIsMovingSupplier.get() ? P_MOVING : P_STATIONARY;
    if (visionP) {
      rotationController.setP(visionPValue);
      rotationController.setD(visionDValue);
    } else {
      if (pCurrent != pDesired) {
        rotationController.setP(pDesired);
        rotationController.setD(0);
      }
    }
    log();
  }

  private void log() {
    if (Constants.DashboardLogging.HEADING) {
      SmartDashboard.putNumber("Heading/kP", rotationController.getP());
      SmartDashboard.putBoolean("Heading/Is Moving", drivetrainIsMovingSupplier.get());

      if (maintainHeading != null) {
        SmartDashboard.putString("Heading/Maintain Heading (Degrees)", String.valueOf(maintainHeading.getDegrees()));
      } else {
        SmartDashboard.putString("Heading/Maintain Heading (Degrees)", "null");
      }
      if (nextHeading != null) {
        SmartDashboard.putString("Heading/Next Heading (Degrees)", String.valueOf(nextHeading.getDegrees()));
      } else {
        SmartDashboard.putString("Heading/Next Heading (Degrees)", "null");
      }
      SmartDashboard.putNumber("Heading/Rotation Controller Error", rotationController.getPositionError());
      SmartDashboard.putBoolean("Heading/Enabled", enabled);
      SmartDashboard.putBoolean("Heading/At Maintain Heading", atMaintainHeading());
    }
  }

  public void enableMaintainHeading() {
    // If we're going from disabled -> enabled, reset our rotation controller
    // to prevent large jumps.
    if (!this.enabled) {
      resetRotationController();
    }
    this.enabled = true;
  }

  public void disableMaintainHeading() {
    this.enabled = false;
  }

  public boolean isEnabled() {
    return this.enabled;
  }

  /**
   * Convert from the gyro angle supplier to a heading rotation value.
   * Gyro angles are reported as CCW rotations being a positive change,
   * and CW rotations being a negative value. We want to convert to a
   * more heading value by flipping those values.
   *
   * @return A Rotation2d representing the heading of the robot.
   */
  private Rotation2d currentHeading() {
    // [-368,640, 368,640] -> (-180, 180)
    return Utilities.convertRotationToRelativeRotation(gyroAngleSupplier.get());
  }

  /**
   * Set the heading for the robot to maintain. If calling this function
   * from a Command, be sure to require the Heading subsystem. Only one
   * Command should be updating the robot's heading at a time.
   *
   * @param maintainHeading The heading the robot should attempt to
   *                        maintain.Expects a value (-180, 180) degrees. Can be
   *                        null if the robot should not maintain a heading.
   */
  public void setMaintainHeading(Rotation2d maintainHeading) {
    if (maintainHeading != null) {
      maintainHeading = Utilities.convertRotationToRelativeRotation(maintainHeading);
    }
    if (this.maintainHeading == maintainHeading) {
      return;
    }
    this.maintainHeading = maintainHeading;
    resetRotationController();
  }

  public Rotation2d getMaintainHeading() {
    return maintainHeading;
  }

  public boolean atMaintainHeading() {
    return rotationController.atSetpoint();
  }

  public boolean shouldMaintainHeading() {
    return this.enabled && maintainHeading != null;
  }

  /**
   * Enqueue the next heading for the robot. There can only be one next
   * heading enqueued at a time. The next heading must be swapped manually
   * to the maintain heading by calling `setNextHeadingToMaintainHeading`.
   *
   * @param nextHeading The next heading the robot should maintain. Expects a
   *                    value from (-180, 180) degrees. Can be null if the robot
   *                    should not maintain a new heading.
   */
  public void setNextHeading(Rotation2d nextHeading) {
    if (nextHeading != null) {
      nextHeading = Utilities.convertRotationToRelativeRotation(nextHeading);
    }
    this.nextHeading = nextHeading;
  }

  /**
   * Set the next heading of the robot to be the robot's new maintained heading.
   * If the robot's next heading is null, the new maintain heading will be null
   * and the robot will not attempt to maintain a heading.
   */
  public void setNextHeadingToMaintainHeading() {
    this.maintainHeading = this.nextHeading;
    this.nextHeading = null;
  }

  /**
   * Uses the rotation PID controller to calculate the
   * rotational value as some percentage from
   * [-max speed, max speed] where max speed is a value
   * we would expect from a joystick (ex: [-1, 1]).
   */
  public double calculateRotation() {
    // If subsystem is disabled - calculateRotation should not be called. Return a
    // 0.0
    if (!this.enabled) {
      // SmartDashboard.putNumber("Heading/Rotation Controller Output", 0.0);
      return 0.0;
    }

    // Should not call `calculateRotation` if `shouldMaintainHeading` is false - but
    // just in case
    if (maintainHeading == null) {
      // SmartDashboard.putNumber("Heading/Rotation Controller Output", 0.0);
      return 0.0;
    }

    Rotation2d currentHeading = currentHeading();
    double output = rotationController.calculate(
        currentHeading.getDegrees(),
        maintainHeading.getDegrees());
    // If our controller is within our tolerance - do not provide a nominal output
    if (rotationController.atSetpoint()) {
      // SmartDashboard.putNumber("Heading/Rotation Controller Output", 0.0);
      return 0.0;
    }
    // Clamp to some max speed (should be between [0.0, 1.0])
    final double maxSpeed = 0.3;
    double clampedOutput = MathUtil.clamp(
        output,
        -maxSpeed,
        maxSpeed);
    double nominalClampedOutput;
    if (visionP) {
      nominalClampedOutput = Math.copySign(
          Math.max(
              Math.abs(clampedOutput),
              drivetrainIsMovingSupplier.get() ? NOMINAL_OUTPUT_MOVING : VISION_NOMINAL_OUTPUT_STATIONARY),
          clampedOutput);
    } else {
      nominalClampedOutput = Math.copySign(
          Math.max(
              Math.abs(clampedOutput),
              drivetrainIsMovingSupplier.get() ? NOMINAL_OUTPUT_MOVING : NOMINAL_OUTPUT_STATIONARY),
          clampedOutput);
    }
    // SmartDashboard.putNumber("Heading/Rotation Controller Output",
    // nominalClampedOutput);
    return nominalClampedOutput;
  }

  /**
   * Resets the state of the internal rotation PID controller.
   * Should be called when the controller's setpoint moves, or
   * when we stop calling `calculate` on a regular basis.
   */
  public void resetRotationController() {
    rotationController.reset();
  }

  // Sets the P Value when we are holding the vision target tracking button.
  public void changePValue(boolean usingVision) {
    visionP = usingVision;
  }

}
