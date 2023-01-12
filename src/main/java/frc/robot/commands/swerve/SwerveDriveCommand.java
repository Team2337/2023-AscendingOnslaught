package frc.robot.commands.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;


public class SwerveDriveCommand extends CommandBase {

  private final XboxController controller;

  private final AutoDrive autoDrive;
  private final Heading heading;
  private final Drivetrain drivetrain;

  // Smooth our our joystick values
  private static double kSlewRateOfChangePerSecond = 1.75;
  SlewRateLimiter forwardSlew = new SlewRateLimiter(kSlewRateOfChangePerSecond);
  SlewRateLimiter strafeSlew = new SlewRateLimiter(kSlewRateOfChangePerSecond);
  SlewRateLimiter rotationSlew = new SlewRateLimiter(kSlewRateOfChangePerSecond);

  /**
   * Command running the swerve calculations with the joystick
   *
   * @param subsystem - SwerveDrivetrain subsystem object
   */
  public SwerveDriveCommand(XboxController controller, AutoDrive autoDrive, Heading heading, Drivetrain drivetrain) {
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.heading = heading;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double forward = -Utilities.deadbandAndSquare(controller.getLeftY());
    double strafe = -Utilities.deadbandAndSquare(controller.getLeftX());
    double rotation = -Utilities.deadbandAndSquare(controller.getRightX(), 0.09);
    boolean isFieldOriented = !controller.getLeftBumper();

    AutoDrive.State autoDriveState = autoDrive.calculate(forward, strafe, isFieldOriented);
    if (autoDriveState != null) {
      forward = autoDriveState.forward;
      strafe = autoDriveState.strafe;
      isFieldOriented = autoDriveState.isFieldOriented;
    }

    if (DriverStation.isTeleopEnabled()) {
      forward = forwardSlew.calculate(forward);
      strafe = strafeSlew.calculate(strafe);
      rotation = rotationSlew.calculate(rotation);
    }
    
    // If a driver-initiated rotation is provided, disable our rotation
    // controller to let the driver rotate freely.
    // Never allow this flow in autonomous (protect against floating joystick
    // values coming from our controllers)
    if (DriverStation.isTeleopEnabled() && rotation != 0 && heading.isEnabled()) {
      heading.disableMaintainHeading();
    }

    /**
     * Calculate a rotation value for the robot to achieve it's
     * maintained heading - if the robot should be maintaining a heading.
     * Will not be calculated if the rotation joystick has an input.
     */
    if (heading.shouldMaintainHeading()) {
      rotation = heading.calculateRotation();
    }

    double vxMetersPerSecond = forward * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    double vyMetersPerSecond = strafe * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    double omegaRadiansPerSecond = rotation * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    if (isFieldOriented) {
      drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          vxMetersPerSecond,
          vyMetersPerSecond,
          omegaRadiansPerSecond,
          drivetrain.getGyroscopeRotation()
        )
      );
    } else {
      if (controller.getLeftBumper()) {
        drivetrain.drive(new ChassisSpeeds(
          -vxMetersPerSecond,
          -vyMetersPerSecond,
          omegaRadiansPerSecond
        ));
        } else {
          drivetrain.drive(new ChassisSpeeds(
          vxMetersPerSecond,
          vyMetersPerSecond,
          omegaRadiansPerSecond
        ));
        }
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
