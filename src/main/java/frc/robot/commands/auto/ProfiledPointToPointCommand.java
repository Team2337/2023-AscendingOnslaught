package frc.robot.commands.auto;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Heading;

/**
 * Generate movement values to drive the robot between it's current position and
 * the specified point. Depends on the robot facing the Hub.
 */
public class ProfiledPointToPointCommand extends HeadingToTargetCommand implements AutoDrivableCommand {

  // These are confirmed tuned values for our Point to Point moves. Can be adjusted
  // individually per move if necessary.
  private static final double forwardP = 1.5;
  private static final double strafeP = 0.05;
  private static final double forwardVelocity = Units.inchesToMeters(160);
  private static final double forwardAcceleration = Units.inchesToMeters(90);
  private static final double strafeVelocity = 45;
  private static final double strafeAcceleration = 12;

  private PolarCoordinate target;
  private Heading heading;
  private AutoDrive autoDrive;

  private ProfiledPIDController distanceController;
  private ProfiledPIDController thetaController;
  private Supplier<Boolean> intakeSensorSupplier;

  private double forwardOutput = 0.0;
  private double strafeOutput = 0.0;

  private boolean monitorIntake = false;
  private boolean endWithIntake = false;

  public ProfiledPointToPointCommand(PolarCoordinate target, Supplier<Translation2d> translationSupplier, AutoDrive autoDrive, Heading heading) {
    this(target, translationSupplier, () -> false, forwardP, strafeP, forwardAcceleration, strafeAcceleration, false, autoDrive, heading);
  }

  public ProfiledPointToPointCommand(PolarCoordinate target, Supplier<Translation2d> translationSupplier, double driveP, double strafeP, double forwardAcceleration, double strafeAcceleration, AutoDrive autoDrive, Heading heading) {
    this(target, translationSupplier, () -> false, forwardP, strafeP, forwardAcceleration, strafeAcceleration, false, autoDrive, heading);
  }

  public ProfiledPointToPointCommand(PolarCoordinate target, Supplier<Translation2d> translationSupplier, Supplier<Boolean> intakeSensorSupplier, double driveP, double strafeP, double forwardAcceleration, double strafeAcceleration, boolean monitorIntake, AutoDrive autoDrive, Heading heading) {
    super(
      target.getReferencePoint(),
      translationSupplier,
      () -> false,
      () -> false,
      null,
      heading,
      null
    );

    this.target = target;
    this.heading = heading;
    this.autoDrive = autoDrive;
    this.monitorIntake = monitorIntake;
    this.intakeSensorSupplier = intakeSensorSupplier;

    distanceController = new ProfiledPIDController(
      driveP, 0.0, 0.0,
      new TrapezoidProfile.Constraints(forwardVelocity, forwardAcceleration)
    );
    thetaController = new ProfiledPIDController(
      strafeP, 0.0, 0.0,
      new TrapezoidProfile.Constraints(strafeVelocity, Math.pow(strafeAcceleration, 2))
    );

    thetaController.enableContinuousInput(-180, 180);

    distanceController.setTolerance(Units.inchesToMeters(1));
    thetaController.setTolerance(0.1); // In degrees

    addRequirements(autoDrive);

    log(getRobotCoordinate());
  }

  @Override
  public void initialize() {
    super.initialize();

    heading.enableMaintainHeading();
    autoDrive.setDelegate(this);

    // Set our initial setpoint for our profiled PID controllers
    // to avoid a JUMP to their starting values on first run
    PolarCoordinate robotCoordinate = getRobotCoordinate();
    thetaController.reset(robotCoordinate.getTheta().getDegrees());
    distanceController.reset(robotCoordinate.getRadiusMeters());
  }

  @Override
  public void execute() {
    super.execute();
    if (monitorIntake) {
      endWithIntake = intakeSensorSupplier.get();
    }

    PolarCoordinate robotCoordinate = getRobotCoordinate();
    forwardOutput = distanceController.calculate(
      robotCoordinate.getRadiusMeters(),
      target.getRadiusMeters()
    );
    strafeOutput = thetaController.calculate(
      robotCoordinate.getTheta().getDegrees(),
      Utilities.convertRotationToRelativeRotation(target.getTheta()).getDegrees()
    );

    // Clamp to some max speed (should be between [0.0, 1.0])
    final double maxSpeed = 1.0;
    forwardOutput = MathUtil.clamp(
      forwardOutput,
      -maxSpeed,
      maxSpeed
    );

    strafeOutput = MathUtil.clamp(
      strafeOutput,
      -maxSpeed,
      maxSpeed
    );

    log(robotCoordinate);
  }

  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    return new AutoDrive.State(
      -forwardOutput,
      -strafeOutput
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    autoDrive.clearDelegate();
  }

  @Override
  public boolean isFinished() {
    return distanceController.atGoal() && thetaController.atGoal() || endWithIntake;
  }

  private void log(PolarCoordinate robotCoordinate) {
    /*
    SmartDashboard.putNumber("ProfiledP2P/Target Distance (inches)", Units.metersToInches(target.getRadiusMeters()));
    SmartDashboard.putNumber("ProfiledP2P/Target Theta (Degrees)", target.getTheta().getDegrees());

    SmartDashboard.putNumber("ProfiledP2P/Robot Angle (degrees)", robotCoordinate.getTheta().getDegrees());
    SmartDashboard.putNumber("ProfiledP2P/Robot Distance (inches)", Units.metersToInches(robotCoordinate.getRadiusMeters()));

    SmartDashboard.putNumber("ProfiledP2P/Forward Output", forwardOutput);
    SmartDashboard.putNumber("ProfiledP2P/Strafe Output", strafeOutput);

    SmartDashboard.putNumber("ProfiledP2P/Distance Position (inches)", Units.metersToInches(distanceController.getSetpoint().position));
    SmartDashboard.putNumber("ProfiledP2P/Distance Error (inches)", Units.metersToInches(distanceController.getPositionError()));

    SmartDashboard.putNumber("ProfiledP2P/Theta Position", thetaController.getSetpoint().position);
    SmartDashboard.putNumber("ProfiledP2P/Theta Error", thetaController.getPositionError());

    SmartDashboard.putBoolean("ProfiledP2P/distanceController atGoal", distanceController.atGoal());
    SmartDashboard.putBoolean("ProfiledP2P/thetaController atGoal", thetaController.atGoal());
    */
  }

}
