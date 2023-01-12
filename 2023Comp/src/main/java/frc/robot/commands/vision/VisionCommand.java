package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public abstract class VisionCommand extends CommandBase {

  protected Drivetrain drivetrain;;
  protected Vision vision;

  /**
   * Get a polar coordinate for the robot based on our vision subsystem.
   * Will return null if we do not see a vision target, or if we are
   * +/- 5 degrees off center of the vision target.
   */
  protected PolarCoordinate calculateVisionPolarCoordiante() {
    // Vision polar coordiante can be determined by using a distance reading
    // + looking at our gyro reading + our tx value.
    // Assumes we're facing the Hub dead-on-ish
    Rotation2d gyroRotation = drivetrain.getGyroscopeRotation();
    Rotation2d hubTheta = gyroRotation.rotateBy(Rotation2d.fromDegrees(180));
    if (vision.hasActiveTarget()) {
      double tx = vision.getTx(); // Degrees from crosshairs center - -29.8 to 29.8
      // If we're more than ~2 degrees off center, we don't have confidence in our
      // hub height reading. Don't calculate an estimated position.
      if (Math.abs(tx) > 2) {
        return null;
      }

      // Commeting our chord math - shouldn't need it if we're facing our target
      // 24 inches = 2 feet - radius of our Hub
      // double chordInches = 2 * Math.sin(Units.degreesToRadians(tx) / 2) * 24;
      // SmartDashboard.putNumber("Chord (inches)", chordInches);

      return new PolarCoordinate(
        vision.getDistanceToCenterHubMeters(),
        hubTheta.minus(Rotation2d.fromDegrees(tx))
      );
    }
    return null;
  }

}
