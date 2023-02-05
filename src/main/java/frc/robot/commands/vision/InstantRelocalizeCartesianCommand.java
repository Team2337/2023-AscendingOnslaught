package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LimelightColor;

public class InstantRelocalizeCartesianCommand extends InstantCommand {
  protected Drivetrain drivetrain;
  protected Vision vision;
  protected LimelightColor color;

  public InstantRelocalizeCartesianCommand(Drivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;

    addRequirements(vision);
  }

  @Override
  public void initialize() {
    if(drivetrain.getPose().getX() < Constants.Vision.VISION_CAMERA_FIELD_ORIENTATION_SWITCHER) {
      color = LimelightColor.BLUE;
    } else {
      color = LimelightColor.ORANGE;
    }

    if (vision.getVisionPose(color).length != 0) {

      double visionPoseX = vision.getVisionPoseX(color);
      double visionPoseY = vision.getVisionPoseY(color);
      double visionRotation = vision.getVisionRotation(color);

      Pose2d pose = new Pose2d(
        new Translation2d(visionPoseX, visionPoseY),
        Rotation2d.fromDegrees(visionRotation)
      );

      drivetrain.resetPosition(pose);
      // vision.incrementRelocalizeCounter();
      
    } 
  }

}
