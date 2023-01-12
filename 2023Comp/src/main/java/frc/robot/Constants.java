package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.nerdyfiles.swerve.configuration.ModuleConfiguration;
import frc.robot.nerdyfiles.swerve.configuration.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static final).  Do not put anything functional in this class.
 *
 * <p>It is advised to static finalally import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static double STARTING_ANGLE = 25;

  public final ModuleConfiguration SWERVE_MODULE_CONFIGURATION;

  public final int MODULE0_DRIVE_MOTOR_ID;
  public final int MODULE0_ANGLE_MOTOR_ID;
  public final int MODULE0_ANGLE_CANCODER_ID;
  public final Rotation2d MODULE0_ANGLE_OFFSET;

  public final int MODULE1_DRIVE_MOTOR_ID;
  public final int MODULE1_ANGLE_MOTOR_ID;
  public final int MODULE1_ANGLE_CANCODER_ID;
  public final Rotation2d MODULE1_ANGLE_OFFSET;

  public final int MODULE2_DRIVE_MOTOR_ID;
  public final int MODULE2_ANGLE_MOTOR_ID;
  public final int MODULE2_ANGLE_CANCODER_ID;
  public final Rotation2d MODULE2_ANGLE_OFFSET;

  public final int MODULE3_DRIVE_MOTOR_ID;
  public final int MODULE3_ANGLE_MOTOR_ID;
  public final int MODULE3_ANGLE_CANCODER_ID;
  public final Rotation2d MODULE3_ANGLE_OFFSET;

  public final double DRIVETRAIN_TRACK_WIDTH_INCHES;
  public final double DRIVETRAIN_WHEEL_BASE_INCHES;

  public final double LIMELIGHT_CAMERA_HEIGHT_METERS;
  public final Rotation2d LIMEILGHT_CAMERA_ANGLE;

  public final int CLIMBER_SLOPE;
  public final int CLIMBER_Y_INTERCEPT;

  public final int CENTERING_BEAM_ID;
  public final int INTAKE_BEAM_ID;

  private static Constants instance;

  public static final class DashboardLogging {
    public static final boolean CLIMBER = false;
    public static final boolean DELIVERY = false;
    public static final boolean DRIVETRAIN = false;
    public static final boolean HEADING = false;
    public static final boolean INTAKE = false;
    public static final boolean KICKER = false;
    public static final boolean PDH = false;
    public static final boolean SHOOTER = false;
    public static final boolean VISION = false;
  }

  // Driver dashboard
  // 28x13
  public static enum DriverDashboardPositions {
    // TODO: could we make this easier by using a method to pass in the widget and one of these enums?
    AUTON_CHOOSER(9, 6, 6, 3),
    STARTING_POS_CHOOSER(9, 0, 6, 3),
    STARTING_ANGLE_CHOOSER(9, 3, 6, 3),
    AUTODRIVE_COMMAND(9, 9, 6, 3),
    GYRO_DEGREES(16, 0, 3, 3),
    ALLIANCE(9, 9, 6, 3),
    INTAKE_BEAM(7, 0, 3, 3),
    DRIVER_CAM(14, 0, 10, 7),
    DISTANCE_TO_TARGET(16, 3, 3, 3);

    public final int x, y, width, height;

    private DriverDashboardPositions(int x, int y, int w, int h) {
      this.x = x;
      this.y = y;
      this.width = w;
      this.height = h;
    }
  }
  public static final ShuffleboardTab DRIVER_DASHBOARD = Shuffleboard.getTab("DRIVER DASHBOARD");

  // Systems check
  public static enum SystemsCheckPositions {
    // Temperatures (3x4 widgets)
    INTAKE_TEMP(0, 0),
    DELIVERY_TEMP(3, 0),
    L_SHOOTER_TEMP(0, 4),
    R_SHOOTER_TEMP(3, 4),
    L_CLIMBER_TEMP(0, 8),
    R_CLIMBER_TEMP(3, 8),
    // Delivery Sensors (3x3 widgets)
    L_COLOR_SENSOR(7, 0),
    R_COLOR_SENSOR(10, 0),
    CENTERING_SENSOR(13, 0),
    // Other sensors (also 3x3 widgets)
    STRING_POT(7, 3),
    PIXY_CAM(10, 3),
    LIMELIGHT(13, 3),
    ROLL_DEGREES(19, 0);

    public final int x, y;

    private SystemsCheckPositions(int x, int y) {
      this.x = x;
      this.y = y;
    }
  }
  public static final boolean DO_SYSTEMS_CHECK = true;
  public static final ShuffleboardTab SYSTEMS_CHECK_TAB = Shuffleboard.getTab("SYSTEMS CHECK");

  public static Constants getInstance() {
    if (instance == null) {
      instance = new Constants();
    }
    return instance;
  }

  public Constants() {
    RobotType.Type robotType = RobotType.getRobotType();
    SmartDashboard.putString("Constants Robot Type", robotType.description);
    switch (robotType) {
      case SKILLSBOT:
        SWERVE_MODULE_CONFIGURATION = SdsModuleConfigurations.MK3_STANDARD;

        MODULE0_DRIVE_MOTOR_ID = 0;
        MODULE0_ANGLE_MOTOR_ID = 4;
        MODULE0_ANGLE_CANCODER_ID = 1;
        MODULE0_ANGLE_OFFSET = Rotation2d.fromDegrees(-50.701904296875);

        MODULE1_DRIVE_MOTOR_ID = 1;
        MODULE1_ANGLE_MOTOR_ID = 5;
        MODULE1_ANGLE_CANCODER_ID = 2;
        MODULE1_ANGLE_OFFSET = Rotation2d.fromDegrees(-128.58123779296875);

        MODULE2_DRIVE_MOTOR_ID = 14;
        MODULE2_ANGLE_MOTOR_ID = 10;
        MODULE2_ANGLE_CANCODER_ID = 3;
        MODULE2_ANGLE_OFFSET = Rotation2d.fromDegrees(-346.63238525390625);

        MODULE3_DRIVE_MOTOR_ID = 15;
        MODULE3_ANGLE_MOTOR_ID = 11;
        MODULE3_ANGLE_CANCODER_ID = 4;
        MODULE3_ANGLE_OFFSET = Rotation2d.fromDegrees(-286.42730712890625);

        DRIVETRAIN_TRACK_WIDTH_INCHES = 10.5;
        DRIVETRAIN_WHEEL_BASE_INCHES = 10.5;
        
        CENTERING_BEAM_ID = 8;
        INTAKE_BEAM_ID = 0;

        LIMELIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(38);
        LIMEILGHT_CAMERA_ANGLE = new Rotation2d(Units.degreesToRadians(34.98));

        CLIMBER_SLOPE = 112676;
        CLIMBER_Y_INTERCEPT = 39538;
        break;
      case PRACTICE:
        SWERVE_MODULE_CONFIGURATION = SdsModuleConfigurations.MK4I_L1;

        MODULE0_DRIVE_MOTOR_ID = 18;
        MODULE0_ANGLE_MOTOR_ID = 19;
        MODULE0_ANGLE_CANCODER_ID = 1;
        MODULE0_ANGLE_OFFSET = Rotation2d.fromDegrees(-131.572);
        MODULE1_DRIVE_MOTOR_ID = 1;
        MODULE1_ANGLE_MOTOR_ID = 2;
        MODULE1_ANGLE_CANCODER_ID = 2;
        MODULE1_ANGLE_OFFSET = Rotation2d.fromDegrees(-175.078);

        MODULE2_DRIVE_MOTOR_ID = 8;
        MODULE2_ANGLE_MOTOR_ID = 9;
        MODULE2_ANGLE_CANCODER_ID = 3;
        MODULE2_ANGLE_OFFSET = Rotation2d.fromDegrees(83.145);

        MODULE3_DRIVE_MOTOR_ID = 10;
        MODULE3_ANGLE_MOTOR_ID = 11;
        MODULE3_ANGLE_CANCODER_ID = 4;
        MODULE3_ANGLE_OFFSET = Rotation2d.fromDegrees(3.779);

        DRIVETRAIN_TRACK_WIDTH_INCHES = 18.75;
        DRIVETRAIN_WHEEL_BASE_INCHES = 18.75;
        
        CENTERING_BEAM_ID = 1;
        INTAKE_BEAM_ID = 0;
        
        LIMELIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(38);
        LIMEILGHT_CAMERA_ANGLE = new Rotation2d(Units.degreesToRadians(30.91193711));

        CLIMBER_SLOPE = 112676;
        CLIMBER_Y_INTERCEPT = 39538;
        break;
      case COMPETITION:
      default:
        SWERVE_MODULE_CONFIGURATION = SdsModuleConfigurations.MK4I_L1;

        MODULE0_DRIVE_MOTOR_ID = 18;
        MODULE0_ANGLE_MOTOR_ID = 19;
        MODULE0_ANGLE_CANCODER_ID = 1;
        MODULE0_ANGLE_OFFSET = Rotation2d.fromDegrees(-76.37109375);

        MODULE1_DRIVE_MOTOR_ID = 1;
        MODULE1_ANGLE_MOTOR_ID = 2;
        MODULE1_ANGLE_CANCODER_ID = 2;
        MODULE1_ANGLE_OFFSET = Rotation2d.fromDegrees(-204.430078125);

        MODULE2_DRIVE_MOTOR_ID = 8;
        MODULE2_ANGLE_MOTOR_ID = 9;
        MODULE2_ANGLE_CANCODER_ID = 3;
        MODULE2_ANGLE_OFFSET = Rotation2d.fromDegrees(-195.37382812500002);

        MODULE3_DRIVE_MOTOR_ID = 10;
        MODULE3_ANGLE_MOTOR_ID = 11;
        MODULE3_ANGLE_CANCODER_ID = 4;
        MODULE3_ANGLE_OFFSET = Rotation2d.fromDegrees(-255.3140625);

        DRIVETRAIN_TRACK_WIDTH_INCHES = 18.75;
        DRIVETRAIN_WHEEL_BASE_INCHES = 18.75;

        CENTERING_BEAM_ID = 8;
        INTAKE_BEAM_ID = 9;

        LIMELIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(37.5); // 38
        LIMEILGHT_CAMERA_ANGLE = new Rotation2d(Units.degreesToRadians(28.43)); // 27.95

        CLIMBER_SLOPE = 171149;
        CLIMBER_Y_INTERCEPT = 49106;
        break;
    }
  }

  /**
   * Sets the Track width and wheel base of the robot based on the centerpoint of the swerve modules.
   * Track width is side to side
   * Wheel base is front to back.
   */
  // /2 since we're measuring from the center - halfway
  private static final double MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES = Constants.getInstance().DRIVETRAIN_TRACK_WIDTH_INCHES / 2;
  private static final double MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES = Constants.getInstance().DRIVETRAIN_WHEEL_BASE_INCHES / 2;

  // Radius to the wheel modules can be thought of as a triangle - width and length are the two sides
  public static final double DRIVETRAIN_RADIUS_INCHES = Math.hypot(MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES, MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES);

  // Location of the Hub on the field - the center of the field
  public static final Translation2d kHub = new Translation2d(
    Units.feetToMeters(27),
    Units.feetToMeters(13.5)
  );
  public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(103.8);
  public static final double VISION_TARGET_OFFSET_FROM_HUB_CENTER_METERS = Units.feetToMeters(2);

  public static final class Auto {
    /**
     * Our polar coordinates for our balls are based off of the center of the field.
     * Note that the angles are measured where the opposing alliance wall
     * is our 0 degrees line (positive X axis). 180 is added to our thetas in order
     * to get them to be on our side of the field, as opposed to the opposing side.
     */
    public static final double kPickupDistanceInches = 8.0;
    public static final double kPickupD1DistanceInches = 44.0;
    public static final double kPickupR2DistanceInches = 34.0;
    public static final double kPickupR3DistanceInches = 34.0;
    public static final double kPickupR4DistanceInches = 14.0;
    public static final double kPickupR5DistanceInches = 26.0;
    public static final double kRunOverDistanceInches = 4.0;
    public static final double kPuntDistanceInches = 4;

    // Starting Locations

    public static final PolarCoordinate kPosition1LeftStart = new PolarCoordinate(
      // Starting angle of -35
      Units.inchesToMeters(97),
      Rotation2d.fromDegrees(147.75)
    );
    public static final PolarCoordinate kPosition2MiddleStart = new PolarCoordinate(
      // Starting angle of 45
      Units.inchesToMeters(94),
      Rotation2d.fromDegrees(-137)
    );
    public static final PolarCoordinate kPosition3RightStart = new PolarCoordinate(
      // Starting angle of 67.42
      Units.inchesToMeters(93),
      Rotation2d.fromDegrees(-99.75)
    );
    public static final PolarCoordinate kPositionFarRightStart = new PolarCoordinate(
      // Starting angle of -90
      Units.inchesToMeters(93),
      Rotation2d.fromDegrees(-90)
    );
    public static final PolarCoordinate kResetToZero = new PolarCoordinate(
      Units.inchesToMeters(133),
      Rotation2d.fromDegrees(180)
    );

    /**
     * Alliance Balls + Shooting Positions
     */

    // Ball R1 = Ball nearest to the left starting location
    public static final PolarCoordinate kBallR1 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(147.75)
    );
    public static final PolarCoordinate kBallR1Pickup = new PolarCoordinate(
      Constants.Auto.kBallR1.getRadiusMeters() - Units.inchesToMeters(kPickupDistanceInches),
      Rotation2d.fromDegrees(147.75)
    );
    public static final PolarCoordinate kBallR1RunOver = new PolarCoordinate(
      Constants.Auto.kBallR1.getRadiusMeters() - Units.inchesToMeters(kRunOverDistanceInches),
      Constants.Auto.kBallR1.getTheta()
    );
    //Shoot postition between ball R2 and ball D2
    public static final PolarCoordinate kFourBallShootPosition = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(-155)
    );
    //Shoot postition between ball R2 and ball D2
    public static final PolarCoordinate kFiveBallShootPosition = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(-155)
    );
    // Ball R2 = Ball nearest to the middle starting location
    public static final PolarCoordinate kBallR2 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(-145.75) //215.25
    );
    public static final PolarCoordinate kBallR2Pickup = new PolarCoordinate(
      Constants.Auto.kBallR2.getRadiusMeters() - Units.inchesToMeters(kPickupR2DistanceInches),
      Constants.Auto.kBallR2.getTheta()
    );
    public static final PolarCoordinate kBallR2RunOver = new PolarCoordinate(
      Constants.Auto.kBallR2.getRadiusMeters() - Units.inchesToMeters(kRunOverDistanceInches),
      Constants.Auto.kBallR2.getTheta()
    );
    // Ball R3 = Ball nearest to the right starting location
    public static final PolarCoordinate kBallR3 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(-99.75)
    );
    public static final PolarCoordinate kBallR3Pickup = new PolarCoordinate(
      Constants.Auto.kBallR3.getRadiusMeters() - Units.inchesToMeters(kPickupDistanceInches),
      Constants.Auto.kBallR3.getTheta()
    );
    public static final PolarCoordinate kBallR3MidPickup = new PolarCoordinate(
      Constants.Auto.kBallR3.getRadiusMeters() - Units.inchesToMeters(kPickupR3DistanceInches),
      Constants.Auto.kBallR3.getTheta()
    );
    public static final PolarCoordinate kBallR3RunOver = new PolarCoordinate(
      Constants.Auto.kBallR3.getRadiusMeters() - Units.inchesToMeters(kRunOverDistanceInches),
      Constants.Auto.kBallR3.getTheta()
    );
    public static final PolarCoordinate kBallR2ShootPosition = new PolarCoordinate(
      Constants.Auto.kBallR2.getRadiusMeters(),
      Constants.Auto.kBallR2.getTheta()
    );
    public static final PolarCoordinate kMidFiveBallShootPosition = new PolarCoordinate(
      Constants.Auto.kBallR2.getRadiusMeters(),
      Rotation2d.fromDegrees(-130)
    );
    public static final PolarCoordinate kBallR4 = new PolarCoordinate(
      Units.inchesToMeters(305.66),
      Rotation2d.fromDegrees(-157.35) //-157.35
    );
    // Point between ball R1 and ball R4 when running four ball auto from left side
    public static final PolarCoordinate TransitionBetweenBallR1AndBallR4 = new PolarCoordinate(
      Units.inchesToMeters(170),
      Rotation2d.fromDegrees(179)
    );
    public static final PolarCoordinate TransitionBetweenBallR2AndBallR4 = new PolarCoordinate(
      Units.inchesToMeters(250),
      Rotation2d.fromDegrees(-149)
    );
    public static final PolarCoordinate kBallR4Pickup = new PolarCoordinate(
      Constants.Auto.kBallR4.getRadiusMeters() - Units.inchesToMeters(kPickupR4DistanceInches),
      Rotation2d.fromDegrees(-153) //-156
    );
    public static final PolarCoordinate kMidFiveBallR4Pickup = new PolarCoordinate(
      Constants.Auto.kBallR4.getRadiusMeters() - Units.inchesToMeters(kPickupR4DistanceInches),
      Rotation2d.fromDegrees(-150.75) //-156
    );
    public static final PolarCoordinate kBallR4PickupForLeftStart = new PolarCoordinate(
      Constants.Auto.kBallR4.getRadiusMeters() - Units.inchesToMeters(kPickupR4DistanceInches),
      Rotation2d.fromDegrees(-161)
    );
    public static final PolarCoordinate kBallR4RunOver = new PolarCoordinate(
      Constants.Auto.kBallR4.getRadiusMeters() - Units.inchesToMeters(kRunOverDistanceInches),
      Rotation2d.fromDegrees(-158.35)
    );
    public static final PolarCoordinate kBallR5Pickup = new PolarCoordinate(
      Constants.Auto.kBallR4.getRadiusMeters() - Units.inchesToMeters(kPickupR5DistanceInches),
      Rotation2d.fromDegrees(-157.35) //-157.35
    );
    public static final PolarCoordinate kBallR7 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(102.75)
    );
    public static final PolarCoordinate kBallR7Pickup = new PolarCoordinate(
      Constants.Auto.kBallR7.getRadiusMeters() - Units.inchesToMeters(kPickupDistanceInches),
      Constants.Auto.kBallR7.getTheta()
    );

    /*
     * Opponent balls
     */
    public static final PolarCoordinate kBallD1 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(127.25)
    );
    public static final PolarCoordinate kBallD1Pickup = new PolarCoordinate(
      Units.inchesToMeters(153) - Units.inchesToMeters(kPickupD1DistanceInches),
      Rotation2d.fromDegrees(127.25)
    );
    public static final PolarCoordinate kBallD1RunOver = new PolarCoordinate(
      Units.inchesToMeters(153) - Units.inchesToMeters(kRunOverDistanceInches),
      Rotation2d.fromDegrees(127.25)
    );
    public static final PolarCoordinate kBallD2 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(-169.95)
    );
    public static final PolarCoordinate kBallD2Punt = new PolarCoordinate(
      Units.inchesToMeters(153) - Units.inchesToMeters(kPuntDistanceInches),
      Rotation2d.fromDegrees(-169.95)
    );
    public static final PolarCoordinate kBallD2Pickup = new PolarCoordinate(
      Units.inchesToMeters(153) - Units.inchesToMeters(kPickupDistanceInches),
      Rotation2d.fromDegrees(-169.95)
    );

    public static final PolarCoordinate kStartAtZero = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(180)
    );
  }

  // Robot-specific configuration for our swerve drive algorithm
  public static final class Swerve {

    public enum ModulePosition {
      FRONT_RIGHT(0),
      FRONT_LEFT(1),
      BACK_LEFT(2),
      BACK_RIGHT(3);

      public final int value;

      ModulePosition(int value) {
        this.value = value;
      }
    }

    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk4 L1 module using Falcon500s to drive.
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4_L1.getDriveReduction() *
      SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

      /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(Units.inchesToMeters(Constants.getInstance().DRIVETRAIN_TRACK_WIDTH_INCHES) / 2.0, Units.inchesToMeters(Constants.getInstance().DRIVETRAIN_WHEEL_BASE_INCHES) / 2.0);
  }

  public static final class Pixy {
    public static final double RATIO_TOLERANCE = 0.4;
  }

  public static final class Vision {
    public static final double IMAGE_PROCESSING_LATENCY_MS = 11;
    public static final double VISION_TARGET_OFFSET_FROM_HUB_CENTER_METERS = Units.feetToMeters(2);
    public static final int RED_PIPELINE_INDEX = 0;
    public static final int BLUE_PIPELINE_INDEX = 1;
  }

  public static final double MOTOR_MINIMUM_TEMP_CELSIUS = 15.0; // Used in Shuffleboard for temperature dials
  public static final double MOTOR_SHUTDOWN_TEMP_CELSIUS = 70.0;

  public static final int CLIMBER_LEFT_MOTOR_ID = 16;
  public static final int CLIMBER_RIGHT_MOTOR_ID = 3;
  public static final int CLIMBER_STRING_POT_ID = 3;
  public static final int LEFT_SERVO_ID = 7;
  public static final int RIGHT_SERVO_ID = 8;

  public static final int KICKER_MOTOR = 20;

  public static final int SHOOTER_LEFT_MOTOR = 7;
  public static final int SHOOTER_RIGHT_MOTOR = 14;

  public static final int DELIVERY_MOTOR_ID = 21;
  public static final double DELIVERY_SPEED = 0.275;
  public static final double BOTTOM_TO_TOP_SPEED = 0.4; //0.35

  public static final int INTAKE_MOTOR_ID = 15;
  public static final double INTAKE_FORWARD_SPEED = 1;
  public static final double INTAKE_REVERSE_SPEED = -0.5;

  public static final int SHOOTER_BEAM_ID = 2;

  public static final int LEDSTRIP_PWM_ID = 1;

  public static final double VISION_TOLERANCE = 1.5;

  public static final String UPPER_CANIVORE_ID = "Upper";

  public static final double VISION_OFFSET = 0;

  public static final double CLIMBER_ROLL = 15;

  public static enum BallColor {
    Red,
    Blue,
    UNKNOWN;

    public static BallColor getAllianceColor() {
      // If we're blue, return blue. Otherwise default to red (if red or invalid).
      return DriverStation.getAlliance() == Alliance.Blue ? Blue : Red;
    }
    public static BallColor getOpposingColor() {
      // The inverse of getAllianceColor
      return DriverStation.getAlliance() == Alliance.Blue ? Red : Blue;
    }
  }

}
