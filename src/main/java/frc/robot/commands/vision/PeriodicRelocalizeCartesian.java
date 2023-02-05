package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LimelightColor;

public class PeriodicRelocalizeCartesian extends VisionCommand {

  private boolean disableRelocalizationInAuto = true;
  private int relocalizationCounterLimit = 5;
  private int relocalizationCounter = 0;
  private LimelightColor color;

  public PeriodicRelocalizeCartesian(Drivetrain drivetrain, Vision vision) {
    this.vision = vision;
    this.drivetrain = drivetrain;

    addRequirements(vision);
  }

  @Override
  public void execute() {
    if(drivetrain.getPose().getX() < Constants.Vision.VISION_CAMERA_FIELD_ORIENTATION_SWITCHER) {
      color = LimelightColor.BLUE;
    } else {
      color = LimelightColor.ORANGE;
    }

    if (relocalizationCounter == relocalizationCounterLimit){
      //TODO: When do we turn this off?
      if (DriverStation.isAutonomous() && disableRelocalizationInAuto) {
        return;
      }
      if (vision.getVisionPose(color).length != 0) {

        double visionPoseX = vision.getVisionPoseX(color);
        double visionPoseY = vision.getVisionPoseY(color);
        double visionRotation = vision.getVisionRotation(color);
      
        Pose2d pose = new Pose2d(
          new Translation2d(visionPoseX, visionPoseY),
          Rotation2d.fromDegrees(visionRotation)
        );

        drivetrain.addVisionMeasurement(pose, Timer.getFPGATimestamp() - ((Constants.Vision.IMAGE_PROCESSING_LATENCY_MS + vision.getLatency(color) + 2) / 1000));
      }
      relocalizationCounter = -1;
    }
    relocalizationCounter++; 
  }
}
