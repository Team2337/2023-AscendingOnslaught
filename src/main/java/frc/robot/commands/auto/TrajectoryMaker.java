package frc.robot.commands.auto;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Vision;

public class TrajectoryMaker extends HeadingToTargetCommand implements AutoDrivableCommand {
    private static final double forwardVelocity = Units.inchesToMeters(160);
    private static final double forwardAcceleration = Units.inchesToMeters(90);
    private Rotation2d startingAngle, endAngle;
    private static final double forwardP = 1.5;
    private static final double strafeP = 0.05;
    private static final double strafeVelocity = 45;
    private static final double strafeAcceleration = 12;
    private double forwardOutput = 0;
    private double strafeOutput = 0;
    private AutoDrive autoDrive;
    private Heading heading;
    private Vision vision;
    private ProfiledPIDController thetaController;
    private SwerveControllerCommand swerveController;
    private Timer timer;
    private Supplier<Translation2d> robotTranslationSupplier;
    private Supplier<Pose2d> poseSupplier;
    private Trajectory newTrajectory;
    private static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(strafeVelocity, strafeAcceleration);
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
    private TrajectoryConfig config = new TrajectoryConfig(forwardVelocity, forwardAcceleration).setKinematics(kinematics);

public TrajectoryMaker(Translation2d target, Supplier<Translation2d> translationSupplier, Supplier<Pose2d> poseSupplier, double x, double y, double a, double b, Rotation2d startingAngle, Rotation2d endAngle, AutoDrive autoDrive, Heading heading, Vision vision) {
   super(
    target,
    translationSupplier,
    () -> false,
    () -> false,
    null,
    heading,
    null
   );

   this.startingAngle = startingAngle;
    this.autoDrive = autoDrive;
    this.heading = heading;
    this.robotTranslationSupplier = robotTranslationSupplier;
    this.poseSupplier = poseSupplier;
    newTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, startingAngle), 
    List.of(new Translation2d(x, y), new Translation2d(a, b)), 
    new Pose2d(x, y, endAngle), 
    config);

    thetaController = new ProfiledPIDController(strafeP, 0, 0, thetaControllerConstraints);
    swerveController = new SwerveControllerCommand(
      newTrajectory, 
      poseSupplier, 
      kinematics, 
      new PIDController(forwardP, 0, 0), 
      new PIDController(strafeP, 0, 0),
      thetaController, 
      null);
    addRequirements(autoDrive);
}

@Override
public void initialize() {
  timer.reset();
  timer.start();
}

@Override
public void execute() {
  double curTime = timer.get();
  var desiredState = newTrajectory.sample(curTime);

 /* var targetChassisSpeeds =
      swerveController.calculate(poseSupplier.get(), desiredState, desiredRotation.get());
  var targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);
*/
  // outputModuleStates.accept(targetModuleStates);
}

@Override
public void end(boolean interrupted) {
  timer.stop();
}

@Override
public boolean isFinished() {
  return timer.hasElapsed(newTrajectory.getTotalTimeSeconds());
}
  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    return new AutoDrive.State(
      -forwardOutput,
      -strafeOutput
    );
  }
}
