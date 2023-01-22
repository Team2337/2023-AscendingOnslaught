package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class InstantRelocalizeCartesianCommand extends InstantCommand {
  protected Drivetrain drivetrain;
  protected Vision vision;

  public InstantRelocalizeCartesianCommand(Drivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;

    addRequirements(vision);
  }

  @Override
  public void initialize() {

    if (vision.getVisionPose().length != 0) {

      double visionPoseX = vision.getVisionPoseX();
      double visionPoseY = vision.getVisionPoseY();
      double visionRotation = vision.getVisionRotation();

      Pose2d pose = new Pose2d(
        new Translation2d(visionPoseX, visionPoseY),
        Rotation2d.fromDegrees(visionRotation)
      );

      drivetrain.resetPosition(pose);
      // vision.incrementRelocalizeCounter();
      
    } 
  }

}
