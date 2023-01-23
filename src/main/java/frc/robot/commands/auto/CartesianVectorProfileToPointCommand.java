package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  // 0.15 = 5" error over 23ft, 14" error over 49ft 
  // 0.1 = 4" error over 23ft, 10" error over 49 ft
  // 0.05 = almost no error, but can oscilate near target
  // TODO: investigate some sort of exponential backoff on the lead (i.e. go from 15% to 100%)
  private static final double intTargetLead = 0.15; // percent, 0.0-1.0

  private Heading heading;
  private AutoDrive autoDrive;

  private ProfiledPIDController driveController;
  private Supplier<Translation2d> translationSupplier;

  private Translation2d startPos;
  private Translation2d target;
  private Translation2d intermediateTarget = new Translation2d();
  private Translation2d driveVector = new Translation2d();


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

    startPos = translationSupplier.get();
    driveController.reset(target.getDistance(startPos));

  }

  @Override
  public void execute() {
    super.execute();
    log();

    // get the distance to the target
    Translation2d currentPos = translationSupplier.get();
    double distanceFromTarget = target.getDistance(currentPos);
    double percentFromTarget = distanceFromTarget / target.getDistance(startPos); // starts at 1.0 and decreases to 0.0

    // target a point xx% between us and the target, interpolated along the original vector to the target
    intermediateTarget = startPos.minus(target).times(percentFromTarget*(1-intTargetLead)).plus(target);
    Translation2d vectorToIntermediateTarget = intermediateTarget.minus(currentPos);

    // turn negative controller output (decrease distance) into positive drive forward
    double driveOutput = -1 * driveController.calculate(
      distanceFromTarget,
      0 // goal = 0 distance to target
    );

    // Clamp to some max power (should be between [0.0, 1.0])
    // final double maxPower = 1.0;
    // driveOutput = MathUtil.clamp(
    //   driveOutput,
    //   -maxPower,
    //   maxPower
    // );

    // turn drive @ angle into forward and strafe values
    driveVector = new Translation2d(driveOutput, vectorToIntermediateTarget.getAngle());
  }

  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    return new AutoDrive.State(
      driveVector.getX(),
      driveVector.getY(),
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
    return driveController.atGoal();
  }

  private void log() {
    SmartDashboard.putNumber("VectorProfile2P/Target X (in)", Units.metersToInches(target.getX()));
    SmartDashboard.putNumber("VectorProfile2P/Target Y (in)", Units.metersToInches(target.getY()));
    SmartDashboard.putNumber("VectorProfile2P/Target Angle (deg)", target.minus(translationSupplier.get()).getAngle().getDegrees());
    SmartDashboard.putNumber("VectorProfile2P/Target Dist (in)", Units.metersToInches(translationSupplier.get().getDistance(target)));

    SmartDashboard.putNumber("VectorProfile2P/Int Target X (in)", Units.metersToInches(intermediateTarget.getX()));
    SmartDashboard.putNumber("VectorProfile2P/Int Target Y (in)", Units.metersToInches(intermediateTarget.getY()));
    SmartDashboard.putNumber("VectorProfile2P/Int Target Angle (deg)", driveVector.getAngle().getDegrees());

    SmartDashboard.putNumber("VectorProfile2P/Robot X (in)", Units.metersToInches(translationSupplier.get().getX()));
    SmartDashboard.putNumber("VectorProfile2P/Robot Y (in)", Units.metersToInches(translationSupplier.get().getY()));

    SmartDashboard.putNumber("VectorProfile2P/Controller Error", driveController.getPositionError());
    SmartDashboard.putBoolean("VectorProfile2P/Controller At Goal", driveController.atGoal());
    
    SmartDashboard.putNumber("VectorProfile2P/Output Drive", driveVector.getNorm());
    SmartDashboard.putNumber("VectorProfile2P/Output Forward", driveVector.getX());
    SmartDashboard.putNumber("VectorProfile2P/Output Strafe", driveVector.getY());
  }

}
