package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class PeriodicRelocalizeCartesian extends VisionCommand {

  // The most recent Vision pose - stored for our scope visualization
  private Pose2d pose;
  private Field2d field = new Field2d();

  // Feed our pose estimator a new vision pose every ~0.2s
  private static final int RELOCALIZE_DEBOUNCE_LIMIT = 10;
  private int relocalizeDebounceCounter = 0;

  public PeriodicRelocalizeCartesian(Drivetrain drivetrain, Vision vision) {
    this.vision = vision;
    this.drivetrain = drivetrain;

    addRequirements(vision);
  }

  @Override
  public void execute() {
    if (vision.getVisionPose().length != 0) {

      double visionPoseX = vision.getVisionPoseX();
      double visionPoseY = vision.getVisionPoseY();
      double visionRotation = vision.getVisionRotation();
      
      Pose2d pose = new Pose2d(
        new Translation2d(visionPoseX, visionPoseY),
        Rotation2d.fromDegrees(visionRotation)
      );

      drivetrain.addVisionMeasurement(pose, Timer.getFPGATimestamp() - ((Constants.Vision.IMAGE_PROCESSING_LATENCY_MS + vision.getLatency() + 2) / 1000));
      // drivetrain.resetPosition(pose);
      // vision.incrementRelocalizeCounter();
      
    } 
  }
}
