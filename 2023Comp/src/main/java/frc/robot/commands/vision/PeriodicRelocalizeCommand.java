package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class PeriodicRelocalizeCommand extends VisionCommand {

  // The most recent Vision pose - stored for our scope visualization
  private Pose2d pose;
  // private Field2d field = new Field2d();

  // Feed our pose estimator a new vision pose every ~0.2s
  private static final int RELOCALIZE_DEBOUNCE_LIMIT = 10;
  private int relocalizeDebounceCounter = 0;

  public PeriodicRelocalizeCommand(Drivetrain drivetrain, Vision vision) {
    this.vision = vision;
    this.drivetrain = drivetrain;

    addRequirements(vision);
  }

  @Override
  public void execute() {
    if (DriverStation.isTeleop()) {
      if (relocalizeDebounceCounter >= RELOCALIZE_DEBOUNCE_LIMIT) {
        PolarCoordinate visionCoordinate = calculateVisionPolarCoordiante();
        if (visionCoordinate != null) {
          pose = new Pose2d(
              visionCoordinate.toFieldCoordinate(),
              drivetrain.getGyroscopeRotation());
          // + 2 because like - a little bonus latency for network stuff
          // / 1000 to go ms -> seconds
          // drivetrain.addVisionMeasurement(
          // pose,
          // Timer.getFPGATimestamp() - ((Constants.Vision.IMAGE_PROCESSING_LATENCY_MS +
          // vision.getLatency() + 2) / 1000)
          // );
          vision.incrementRelocalizeCounter();

          relocalizeDebounceCounter = 0;
        }
      } else {
        relocalizeDebounceCounter++;
      }

      if (pose != null) {
        // field.setRobotPose(pose);
        /*
         * Logger.getInstance().recordOutput("Vision/Robot",
         * new double[] { pose.getX(), pose.getY(),
         * pose.getRotation().getRadians() });
         */
      }
    }
  }

}
