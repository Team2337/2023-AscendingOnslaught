package frc.robot.nerdyfiles.vision;

import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightUtilities {
  /**
   * Algorithm from
   * https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   * Estimates
   * range to a target using the target's elevation. This method can produce more
   * stable results
   * than SolvePNP when well tuned, if the full 6d robot pose is not required.
   * Note that this method
   * requires the camera to have 0 roll (not be skewed clockwise or CCW relative
   * to the floor), and
   * for there to exist a height differential between goal and camera. The larger
   * this differential,
   * the more accurate the distance estimate will be.
   *
   * <p>
   * Units can be converted using the {@link edu.wpi.first.math.util.Units}
   * class.
   *
   * @param cameraHeightMeters The physical height of the camera off the floor in
   *                           meters.
   * @param targetHeightMeters The physical height of the target off the floor in
   *                           meters. This
   *                           should be the height of whatever is being targeted
   *                           (i.e. if the targeting region is set to
   *                           top, this should be the height of the top of the
   *                           target).
   * @param cameraPitch        The pitch of the camera from the horizontal plane.
   *                           Positive values up.
   * @param targetPitch        The pitch of the target in the camera's lens.
   *                           Positive values up.
   * @param targetYaw          The observed yaw of the target. Note that this
   *                           *must* be CCW-positive.
   * @return The estimated distance to the target in meters.
   */
  public static double calculateDistanceToTargetMeters(
    double cameraHeightMeters,
    double targetHeightMeters,
    Rotation2d cameraPitch,
    Rotation2d targetPitch,
    Rotation2d targetYaw) {
    return (targetHeightMeters - cameraHeightMeters) / (Math.tan(cameraPitch.getRadians() + targetPitch.getRadians()) * targetYaw.getCos());
  }

}
