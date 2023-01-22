package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class PeriodicRelocalizeCartesian extends VisionCommand {

  private boolean disableRelocalizationInAuto = true;

  public PeriodicRelocalizeCartesian(Drivetrain drivetrain, Vision vision) {
    this.vision = vision;
    this.drivetrain = drivetrain;

    addRequirements(vision);
  }

  @Override
  public void execute() {
    //TODO: When do we turn this off?
    if (DriverStation.isAutonomous() && disableRelocalizationInAuto) {
      return;
    }
    if (vision.getVisionPose().length != 0) {

      double visionPoseX = vision.getVisionPoseX();
      double visionPoseY = vision.getVisionPoseY();
      double visionRotation = vision.getVisionRotation();
      
      Pose2d pose = new Pose2d(
        new Translation2d(visionPoseX, visionPoseY),
        Rotation2d.fromDegrees(visionRotation)
      );

      drivetrain.addVisionMeasurement(pose, Timer.getFPGATimestamp() - ((Constants.Vision.IMAGE_PROCESSING_LATENCY_MS + vision.getLatency() + 2) / 1000));
    } 
  }
}
