package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class InstantRelocalizeCommand extends VisionCommand {

  public InstantRelocalizeCommand(Drivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;

    addRequirements(vision);
  }

  @Override
  public void initialize() {
    PolarCoordinate visionCoordinate = calculateVisionPolarCoordiante();
    if (visionCoordinate != null) {
      Pose2d pose = new Pose2d(
        visionCoordinate.toFieldCoordinate(),
        drivetrain.getGyroscopeRotation()
      );
      drivetrain.resetPosition(pose);
      vision.incrementRelocalizeCounter();
    }
  }

}
