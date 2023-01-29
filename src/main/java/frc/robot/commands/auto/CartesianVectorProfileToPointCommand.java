package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CartesianHeadingToTargetCommand;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Heading;

/**
 * Generate movement values to drive the robot between it's current position and
 * the specified point.
 */
public class CartesianVectorProfileToPointCommand extends CartesianHeadingToTargetCommand implements AutoDrivableCommand {

  // These are confirmed tuned values for our Point to Point moves. Can be adjusted
  // individually per move if necessary.
  private static final double maxVelocity = Units.inchesToMeters(162);

  private Translation2d target;
  private Heading heading;
  private AutoDrive autoDrive;

  private ProfiledPIDController driveController;
  private Supplier<Translation2d> translationSupplier;

  private double driveOutput = 0;
  private double forwardOutput = 0.0;
  private double strafeOutput = 0.0;

  public CartesianVectorProfileToPointCommand(
    Translation2d target,
    Supplier<Translation2d> translationSupplier,
    double driveP,
    double maxAcceleration,
    AutoDrive autoDrive,
    Heading heading
  ) {
    super(
      new Translation2d(0, 0),
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
    this.translationSupplier = translationSupplier;

    driveController = new ProfiledPIDController(
      driveP, 0.0, 0.0,
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
    );

    driveController.setTolerance(Units.inchesToMeters(2));

    log();

    addRequirements(autoDrive);
  }

  @Override
  public void initialize() {
    super.initialize();

    heading.enableMaintainHeading();
    autoDrive.setDelegate(this);

    // Set our initial setpoint for our profiled PID controllers
    // to avoid a JUMP to their starting values on first run
    Translation2d robotCoordinate = translationSupplier.get();
    driveController.reset(robotCoordinate.getDistance(target));

  }

  @Override
  public void execute() {
    super.execute();
    log();
    Translation2d robotCoordinate = translationSupplier.get();
    double forwardDiff = target.getX() - robotCoordinate.getX();
    double strafeDiff = target.getY() - robotCoordinate.getY();
    double degreesToTarget = Math.atan2(strafeDiff,forwardDiff);

    driveOutput = -driveController.calculate(
      robotCoordinate.getDistance(target),
      0 // goal = 0 distance to target
    );

    // Clamp to some max speed (should be between [0.0, 1.0])
    final double maxSpeed = 1.0;
    driveOutput = MathUtil.clamp(
      driveOutput,
      -maxSpeed,
      maxSpeed
    );

    forwardOutput = driveOutput * Math.cos(degreesToTarget);
    strafeOutput = driveOutput * Math.sin(degreesToTarget);
  }

  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    return new AutoDrive.State(
      forwardOutput,
      strafeOutput,
      true
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    autoDrive.clearDelegate();
  }

  @Override
  public boolean isFinished() {
    return  Units.metersToInches(translationSupplier.get().getDistance(target)) < 1;
  }

  private void log() {
    
    SmartDashboard.putNumber("VectorProfile2P/Target X (in)", Units.metersToInches(target.getX()));
    SmartDashboard.putNumber("VectorProfile2P/Target Y (in)", Units.metersToInches(target.getY()));

    SmartDashboard.putNumber("VectorProfile2P/Robot X (in)", Units.metersToInches(translationSupplier.get().getX()));
    SmartDashboard.putNumber("VectorProfile2P/Robot Y (in)", Units.metersToInches(translationSupplier.get().getY()));

    SmartDashboard.putNumber("VectorProfile2P/Drive Output", driveOutput);
    SmartDashboard.putNumber("VectorProfile2P/Drive Error (in)", Units.metersToInches(driveController.getPositionError()));
    SmartDashboard.putNumber("VectorProfile2P/Dist to Target (in)", Units.metersToInches(translationSupplier.get().getDistance(target)));
    SmartDashboard.putBoolean("VectorProfile2P/forwardController atGoal", driveController.atGoal());
    
    SmartDashboard.putNumber("VectorProfile2P/Forward Output", forwardOutput);
    SmartDashboard.putNumber("VectorProfile2P/Strafe Output", strafeOutput);
  }

}
