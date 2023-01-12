package frc.robot.coordinates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;

public class PolarCoordinate {

  private double radiusMeters;
  private Rotation2d theta;
  // The reference point is the "center" in our polar coordinate system
  private Translation2d referencePoint;

  /**
   * Construct a polar coordinate using the center of the field (the Hub)
   * as the reference point.
   *
   * @param radiusMeters - The radius from the reference point in meters.
   * @param theta        - The rotation from the reference point. In our
   *                     field-centric system, this is the left-hand side.
   */
  public PolarCoordinate(double radiusMeters, Rotation2d theta) {
    this(radiusMeters, theta, Constants.kHub);
  }

  /**
   * Construct a polar coordinate using a specific center point.
   *
   * @param radiusMeters   - The radius from the reference point in meters.
   * @param theta          - The rotation from the reference point. In our
   *                       field-centric system, this is the left-hand side.
   * @param referencePoint - The x, y to use as the center for our polar
   *                       coordinate. Should be a field coordinate.
   */
  public PolarCoordinate(double radiusMeters, Rotation2d theta, Translation2d referencePoint) {
    this.radiusMeters = radiusMeters;
    this.theta = theta;
    this.referencePoint = referencePoint;
  }

  /**
   * Create a PolarCoordinate from a field coordinate with using the center of the
   * field (the Hub)
   *
   * @param coordinate - The field coordinate to translate from
   * @return A polar coordinate with the hypotenuse as the distance and the atan
   *         as the theta. Theta will be in (-180, 180) range.
   */
  public static PolarCoordinate fromFieldCoordinate(Translation2d coordinate) {
    return fromFieldCoordinate(coordinate, Constants.kHub);
  }

  /**
   * Create a PolarCoordinate from a field coordinate and a given field coordinate
   * reference point.
   *
   * @param coordinate     - The field coordinate to translate from
   * @param referencePoint - The field coordinate reference point for the polar
   *                       coordiate
   * @return A polar coordinate with the hypotenuse as the distance and the atan
   *         as the theta. Theta will be in (-180, 180) range.
   */
  public static PolarCoordinate fromFieldCoordinate(Translation2d coordinate, Translation2d referencePoint) {
    double x = coordinate.getX() - referencePoint.getX();
    double y = coordinate.getY() - referencePoint.getY();
    // Distance is our hypotenuse of our triangle
    // Angle is our tangent of our two components
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    double angle = Math.atan2(y, x); // (-π, π] radians
    return new PolarCoordinate(
      distance,
      new Rotation2d(angle),
      referencePoint
    );
  }

  public double getRadiusMeters() {
    return radiusMeters;
  }

  public Rotation2d getTheta() {
    return theta;
  }

  public Translation2d getReferencePoint() {
    return referencePoint;
  }

  /**
   * Translates our polar coordinate to a x, y in a cartestian
   * coordinate system relative to our reference point on the field.
   *
   * @return A x, y of our polar coordinate represented in a cartesian coordinate
   *         system relative to the reference point.
   */
  public Translation2d toFieldCoordinate() {
    double x = radiusMeters * Math.cos(theta.getRadians());
    double y = radiusMeters * Math.sin(theta.getRadians());
    return new Translation2d(
      referencePoint.getX() + x,
      referencePoint.getY() + y
    );
  }

  /**
   * Rotate our coordinate around the reference point by some
   * rotational value.
   *
   * Ex: Rotating 45 degrees by 90 -> 135 degrees
   * Ex: Rotating 45 degrees by -90 -> -45 degrees
   * Ex: Rotating 90 degrees by 180 -> -90 degrees
   */
  public PolarCoordinate rotateBy(Rotation2d other) {
    return new PolarCoordinate(
      radiusMeters,
      theta.rotateBy(other),
      referencePoint
    );
  }

  /**
   * Returns a new PolarCoordinate with rotational value
   * constrainted to a single-rotation `(-180, 180)` range
   */
  public PolarCoordinate withRelativeTheta() {
    return new PolarCoordinate(
      radiusMeters,
      Utilities.convertRotationToRelativeRotation(theta),
      referencePoint
    );
  }

}
