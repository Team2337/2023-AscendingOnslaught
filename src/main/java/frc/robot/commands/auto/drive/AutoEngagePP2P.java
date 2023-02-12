package frc.robot.commands.auto.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
 * the specified point. Depends on the robot facing the Hub.
 */
public class AutoEngagePP2P extends CartesianHeadingToTargetCommand implements AutoDrivableCommand {

  // These are confirmed tuned values for our Point to Point moves. Can be adjusted
  // individually per move if necessary.
  private static final double forwardVelocity = Units.inchesToMeters(160);
  private static final double strafeVelocity = Units.inchesToMeters(160);

  private Translation2d target;
  private Heading heading;
  private AutoDrive autoDrive;

  private ProfiledPIDController forwardController;
  private ProfiledPIDController strafeController;
  private Supplier<Translation2d> translationSupplier;
  private Supplier<Rotation2d> rotationSupplier;
  private Supplier<Rotation2d> pitchSupplier;

  private double forwardOutput = 0.0;
  private double strafeOutput = 0.0;

  private boolean isElevated = false; 

  public AutoEngagePP2P(
    Translation2d target,
    Supplier<Translation2d> translationSupplier,
    Supplier<Rotation2d> rotationSupplier,
    double driveP,
    double strafeP,
    double forwardAcceleration,
    double strafeAcceleration,
    Supplier<Rotation2d> pitchSupplier,
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
    this.rotationSupplier = rotationSupplier;
    this.pitchSupplier = pitchSupplier;

    forwardController = new ProfiledPIDController(
      driveP, 0.0, 0.0,
      new TrapezoidProfile.Constraints(forwardVelocity, forwardAcceleration)
    );
    strafeController = new ProfiledPIDController(
      strafeP, 0.0, 0.0,
      new TrapezoidProfile.Constraints(strafeVelocity, strafeAcceleration)
    );

    forwardController.setTolerance(Units.inchesToMeters(2));
    strafeController.setTolerance(Units.inchesToMeters(2));

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
    strafeController.reset(robotCoordinate.getY());
    forwardController.reset(robotCoordinate.getX());
   // currentRotation = rotationSupplier.get().getRadians();

  }

  @Override
  public void execute() {
    super.execute();
    log();
    Translation2d robotCoordinate = translationSupplier.get();
    // currentRotation = rotationSupplier.get().getRadians();
  
    forwardOutput = forwardController.calculate(
      robotCoordinate.getX(),
      target.getX()
    );
    strafeOutput = strafeController.calculate(
      robotCoordinate.getY(),
      target.getY()
    );

    if (pitchSupplier.get().getDegrees() < -10) {
      isElevated = true;
    }

    // Replaced with isFieldOriented = true on AutoDrive.State
    // rotatedForwardOutput = forwardOutput * Math.cos(currentRotation) + strafeOutput * Math.sin(currentRotation);
    // rotatedStrafeOutput = strafeOutput * Math.cos(currentRotation) - forwardOutput * Math.sin(currentRotation);

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
    return (forwardController.atGoal() && strafeController.atGoal()) || (isElevated && pitchSupplier.get().getDegrees() > -8.5);
  }

  private void log() {
    
    SmartDashboard.putNumber("ProfiledP2P/Target Forward (inches)", Units.metersToInches(target.getX()));
    SmartDashboard.putNumber("ProfiledP2P/Target Strafe (inches)", Units.metersToInches(target.getY()));

    SmartDashboard.putNumber("ProfiledP2P/Robot Forward X (inches)", Units.metersToInches(translationSupplier.get().getX()));
    SmartDashboard.putNumber("ProfiledP2P/Robot Strafe Y (inches)", Units.metersToInches(translationSupplier.get().getY()));

    SmartDashboard.putNumber("ProfiledP2P/Forward Output", forwardOutput);
    SmartDashboard.putNumber("ProfiledP2P/Strafe Output", strafeOutput);

    SmartDashboard.putNumber("ProfiledP2P/Forward Error (inches)", Units.metersToInches(forwardController.getPositionError()));
    SmartDashboard.putNumber("ProfiledP2P/Strafe Error (inches)", Units.metersToInches(strafeController.getPositionError()));

    SmartDashboard.putBoolean("ProfiledP2P/forwardController atGoal", forwardController.atGoal());
    SmartDashboard.putBoolean("ProfiledP2P/strafeController atGoal", strafeController.atGoal());
    
  }

}
