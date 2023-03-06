package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public static final boolean ARM = true;
    public static final boolean AUTO = false;
    public static final boolean ELBOW = true;
    public static final boolean DRIVETRAIN = false;
    public static final boolean HEADING = false;
    public static final boolean INTAKE = false;
    public static final boolean INTAKESPINNER = true;
    public static final boolean PDH = false;
    public static final boolean SHOULDER = true;
    public static final boolean SWERVE = false;
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
    SmartDashboard.putString("Startup/Mac-Address", RobotType.getMACAddress());
    SmartDashboard.putString("Startup/Robot Type", robotType.description);
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
        MODULE0_ANGLE_OFFSET = Rotation2d.fromDegrees(94.658203125);//94.39);

        MODULE1_DRIVE_MOTOR_ID = 1;
        MODULE1_ANGLE_MOTOR_ID = 2;
        MODULE1_ANGLE_CANCODER_ID = 2;
        MODULE1_ANGLE_OFFSET = Rotation2d.fromDegrees(-7.20703125);//-5.8867);

        MODULE2_DRIVE_MOTOR_ID = 8;
        MODULE2_ANGLE_MOTOR_ID = 9;
        MODULE2_ANGLE_CANCODER_ID = 3;
        MODULE2_ANGLE_OFFSET = Rotation2d.fromDegrees(48.8671875);//48.259153);

        MODULE3_DRIVE_MOTOR_ID = 10;
        MODULE3_ANGLE_MOTOR_ID = 11;
        MODULE3_ANGLE_CANCODER_ID = 4;
        MODULE3_ANGLE_OFFSET = Rotation2d.fromDegrees(1.669921875);//2.7246);

        DRIVETRAIN_TRACK_WIDTH_INCHES = 19.75;
        DRIVETRAIN_WHEEL_BASE_INCHES = 24.75;
        
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
        MODULE0_ANGLE_OFFSET = Rotation2d.fromDegrees(66.796875);

        MODULE1_DRIVE_MOTOR_ID = 1;
        MODULE1_ANGLE_MOTOR_ID = 2;
        MODULE1_ANGLE_CANCODER_ID = 2;
        MODULE1_ANGLE_OFFSET = Rotation2d.fromDegrees(73.564453125);

        MODULE2_DRIVE_MOTOR_ID = 8;
        MODULE2_ANGLE_MOTOR_ID = 9;
        MODULE2_ANGLE_CANCODER_ID = 3;
        MODULE2_ANGLE_OFFSET = Rotation2d.fromDegrees(104.23828125);

        MODULE3_DRIVE_MOTOR_ID = 10;
        MODULE3_ANGLE_MOTOR_ID = 11;
        MODULE3_ANGLE_CANCODER_ID = 4;
        MODULE3_ANGLE_OFFSET = Rotation2d.fromDegrees(103.7109375);

        DRIVETRAIN_TRACK_WIDTH_INCHES = 19.75;
        DRIVETRAIN_WHEEL_BASE_INCHES = 24.75;

        CENTERING_BEAM_ID = 8;
        INTAKE_BEAM_ID = 9;

        LIMELIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(25.5); // 38
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
    public static final double lengthOfField = Units.inchesToMeters(678.42);
    public static final double heightOfField = Units.inchesToMeters(315.6);
    public static final double hybridNodeLength = Units.inchesToMeters(14.28);
    public static final double centerOfRobotWidth = Units.inchesToMeters(15.5);
    public static final double centerOfRobotLength = Units.inchesToMeters(18);
    public static final double robotOffsetFromHybridAndPickupNodes = Units.inchesToMeters(21.5);
    public static final double floorPickupArmReach = Units.inchesToMeters(12);
    public static final double robotChargeStationYOffset = Units.inchesToMeters(24);
    public static final double chargeAutoStationOffset = Units.inchesToMeters(24);
    public static final double trajectoryCutoff = 24;
    public static final double trajectoryTolerance = 1;
    public static final double intakeAutoForwardSpeed = 1.0;
    public static final double intakeAutoReverseSpeed = -0.5;
    public static final double robotScoringOffset = Units.inchesToMeters(2);
    public static final double pickupOffset = Units.inchesToMeters(14);

    // Blue April Tag Locations
    public static final Translation2d blueTop6 = new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19));
    public static final Translation2d blueMiddle7 = new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19));
    public static final Translation2d blueBottom8 = new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19));
    public static final Translation2d blueSubstation4 = new Translation2d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74));

    // Red April Tag Locations
    public static final Translation2d redTop1 = new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(273.41));
    public static final Translation2d redMiddle2 = new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(207.43));
    public static final Translation2d redBottom3 = new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(141.41));
    public static final Translation2d redSubstation5 = new Translation2d(Units.inchesToMeters(636.96), Units.inchesToMeters(49.86));

    public static final Translation2d zeroPoint = new Translation2d(0, 0);

    // Blue Starting Locations
    public static final Translation2d blueGridLeftRobotLeft = new Translation2d(blueTop6.getX() + centerOfRobotLength + hybridNodeLength, blueTop6.getY() + robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d blueGridLeftRobotCenter = new Translation2d(blueTop6.getX() + centerOfRobotLength + hybridNodeLength, blueTop6.getY());
    public static final Translation2d blueGridLeftRobotRight = new Translation2d(blueTop6.getX() + centerOfRobotLength + hybridNodeLength, blueTop6.getY() - robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d blueGridMiddleRobotLeft = new Translation2d(blueMiddle7.getX() + centerOfRobotLength + hybridNodeLength, blueMiddle7.getY() + robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d blueGridMiddleRobotCenter = new Translation2d(blueMiddle7.getX() + centerOfRobotLength + hybridNodeLength, blueMiddle7.getY());
    public static final Translation2d blueGridMiddleRobotRight = new Translation2d(blueMiddle7.getX() + centerOfRobotLength + hybridNodeLength, blueMiddle7.getY() - robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d blueGridRightRobotLeft = new Translation2d(blueBottom8.getX() + centerOfRobotLength + hybridNodeLength, blueBottom8.getY() + robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d blueGridRightRobotCenter = new Translation2d(blueBottom8.getX() + centerOfRobotLength + hybridNodeLength, blueBottom8.getY());
    public static final Translation2d blueGridRightRobotRight = new Translation2d(blueBottom8.getX() + centerOfRobotLength + hybridNodeLength, blueBottom8.getY() - robotOffsetFromHybridAndPickupNodes);

    // Red Starting Locations
    public static final Translation2d redGridLeftRobotLeft = new Translation2d(redTop1.getX() + centerOfRobotLength + hybridNodeLength + robotScoringOffset, redTop1.getY() + robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d redGridLeftRobotCenter = new Translation2d(redTop1.getX() + centerOfRobotLength + hybridNodeLength, redTop1.getY());
    public static final Translation2d redGridLeftRobotRight = new Translation2d(redTop1.getX() + centerOfRobotLength + hybridNodeLength, redTop1.getY() - robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d redGridMiddleRobotLeft = new Translation2d(redMiddle2.getX() + centerOfRobotLength + hybridNodeLength, redMiddle2.getY() + robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d redGridMiddleRobotCenter = new Translation2d(redMiddle2.getX() + centerOfRobotLength + hybridNodeLength + robotScoringOffset, redMiddle2.getY());
    public static final Translation2d redGridMiddleRobotRight = new Translation2d(redMiddle2.getX() + centerOfRobotLength + hybridNodeLength, redMiddle2.getY() - robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d redGridRightRobotLeft = new Translation2d(redBottom3.getX() + centerOfRobotLength + hybridNodeLength, redBottom3.getY() + robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d redGridRightRobotCenter = new Translation2d(redBottom3.getX() + centerOfRobotLength + hybridNodeLength, redBottom3.getY());
    public static final Translation2d redGridRightRobotRight = new Translation2d(redBottom3.getX() + centerOfRobotLength + hybridNodeLength + robotScoringOffset, redBottom3.getY() - robotOffsetFromHybridAndPickupNodes); 

    public static final Translation2d redRightyRight = new Translation2d(Units.inchesToMeters(74), Units.inchesToMeters(116.1));
    
    // Blue Staging Marks
    public static final Translation2d blueBottomStagingMark = new Translation2d(Units.inchesToMeters(278.05) - centerOfRobotLength - floorPickupArmReach, Units.inchesToMeters(36.19));
    public static final Translation2d blueMiddleStagingMark = new Translation2d(Units.inchesToMeters(278.05) - centerOfRobotLength - floorPickupArmReach, Units.inchesToMeters(84.19));
    public static final Translation2d blueTopStagingMark = new Translation2d(Units.inchesToMeters(278.05) - centerOfRobotLength - floorPickupArmReach, Units.inchesToMeters(132.19));
    public static final Translation2d blueToppyTopStagingMark = new Translation2d(Units.inchesToMeters(278.05) - centerOfRobotLength + pickupOffset, Units.inchesToMeters(180.19));

    public static final Translation2d partnerShowcasePickup = new Translation2d(Units.inchesToMeters(207.73), Units.inchesToMeters(-22));
    public static final Translation2d partnerShowcaseScore = new Translation2d(blueBottom8.getX() + centerOfRobotLength + hybridNodeLength, Units.inchesToMeters(6.000));

    // Red Staging Marks
    public static final Translation2d redBottomStagingMark = new Translation2d(Units.inchesToMeters(278.05) - centerOfRobotLength + pickupOffset, Units.inchesToMeters(135.41));
    public static final Translation2d redMiddleStagingMark = new Translation2d(Units.inchesToMeters(278.05) - centerOfRobotLength - floorPickupArmReach, Units.inchesToMeters(173.61));
    public static final Translation2d redTopStagingMark = new Translation2d(Units.inchesToMeters(278.05) - centerOfRobotLength - floorPickupArmReach, Units.inchesToMeters(221.61));
    public static final Translation2d redToppyTopStagingMark = new Translation2d(Units.inchesToMeters(278.05) - centerOfRobotLength - floorPickupArmReach, Units.inchesToMeters(269.61));

    // Blue Substation Pickup Locations
    public static final Translation2d blueLeftSubstationPickup = new Translation2d(blueSubstation4.getX() - floorPickupArmReach - centerOfRobotLength, blueSubstation4.getY() + robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d blueRightSubstationPickup = new Translation2d(blueSubstation4.getX() - floorPickupArmReach - centerOfRobotLength, blueSubstation4.getY() - robotOffsetFromHybridAndPickupNodes);

    // Red Substation Pickup Locations
    public static final Translation2d redLeftSubstationPickup = new Translation2d(redSubstation5.getX() - floorPickupArmReach - centerOfRobotLength, redSubstation5.getY() + robotOffsetFromHybridAndPickupNodes);
    public static final Translation2d redRightSubstationPickup = new Translation2d(redSubstation5.getX() - floorPickupArmReach - centerOfRobotLength, redSubstation5.getY() - robotOffsetFromHybridAndPickupNodes);

    // Blue Charge Station
    public static final Translation2d blueCenterOfChargeStation = new Translation2d(Units.inchesToMeters(150.8) + chargeAutoStationOffset, Units.inchesToMeters(108.19));
    public static final Translation2d blueLeftCenterOfChargeStation = new Translation2d(Units.inchesToMeters(150.8), Units.inchesToMeters(108.19) + robotChargeStationYOffset);
    public static final Translation2d blueRightCenterOfChargeStation = new Translation2d(Units.inchesToMeters(150.8), Units.inchesToMeters(108.19) - robotChargeStationYOffset);

    // Red Charge Station
    public static final Translation2d redCenterOfChargeStation = new Translation2d(Units.inchesToMeters(150.8), Units.inchesToMeters(207.41));
    public static final Translation2d redLeftCenterOfChargeStation = new Translation2d(Units.inchesToMeters(150.8), Units.inchesToMeters(207.41) + robotChargeStationYOffset);
    public static final Translation2d redRightCenterOfChargeStation = new Translation2d(Units.inchesToMeters(150.8), Units.inchesToMeters(207.41) - robotChargeStationYOffset);

    // Blue Intermediary Points
    public static final Translation2d blueLeftIntermediaryNear = new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(185));
    public static final Translation2d blueLeftIntermediaryFar = new Translation2d(Units.inchesToMeters(190), Units.inchesToMeters(185));
    public static final Translation2d blueRightIntermediaryNear = new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(30));
    public static final Translation2d blueRightIntermediaryAutoNear = new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(30));
    public static final Translation2d blueRightIntermediaryFar = new Translation2d(Units.inchesToMeters(190), Units.inchesToMeters(30));
    public static final Translation2d blueSubstationIntermediary = new Translation2d(blueSubstation4.getX() - Units.inchesToMeters(100), blueSubstation4.getY() + centerOfRobotWidth);

    // Red Intermediary Points
    public static final Translation2d redLeftIntermediaryNear = new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(285));
    public static final Translation2d redLeftIntermediaryFar = new Translation2d(Units.inchesToMeters(190), Units.inchesToMeters(285));
    public static final Translation2d redRightIntermediaryNear = new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(130));
    public static final Translation2d redRightIntermediaryFar = new Translation2d(Units.inchesToMeters(190), Units.inchesToMeters(130));
    public static final Translation2d redSubstationIntermediary = new Translation2d(redSubstation5.getX() - Units.inchesToMeters(100), redSubstation5.getY() + centerOfRobotWidth);
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
    //TODO: Change to 7.5
    public static final double VISION_CAMERA_FIELD_ORIENTATION_SWITCHER = 7.5;
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

  public static final int INTAKE_MOTOR_ID = 17;
  public static final double INTAKE_FORWARD_SPEED = 1;
  public static final double INTAKE_REVERSE_SPEED = -0.5;

  public static final int SHOOTER_BEAM_ID = 2;

  public static final int LEDSTRIP_PWM_ID = 9;

  public static final double VISION_TOLERANCE = 1.5;

  public static final String UPPER_CANIVORE_ID = "Upper";

  public static final double VISION_OFFSET = 0;

  public static final double CLIMBER_ROLL = 15;
  public static class Arm {
    public static final double SHOULDER_ARM_LENGTH = 25;
    public static final double ELBOW_ARM_LENGTH = 34;
    public static final double ARM_GEAR_RATIO = 218.75;
    public static final double shoulderP = 0.005;
    public static final double shoulderI = 0.0;
    public static final double shoulderD = 0.0;
    public static final double WRIST_LOWER_LIMIT = 23.0;
    public static final double WRIST_UPPER_LIMIT = 260.0;
    public static final double elbowP = 0.2;
    public static final double elbowI = 0.0;
    public static final double elbowD = 0.0;
    public static final double intakeP = 0.2;
    public static final double intakeI = 0.0;
    public static final double intakeD = 0.0;

    // Comp Arm
    public enum ArmPosition {
      SCOREHIGH(109.0,40.0,109.0,54.0,161.0,23.0),
      SCOREMID(92.0,89.0,109.0,78.0,165.0,23.0),
      SCORELOW(100.0,145.0,100.0,145.0,185.0,75.0),
      SUBSTATION(65.0,-55.0,66.0,-61.0,255.0,114.0),
      TELESTANDINGCONE(55.0,-118.0,35.0,-72.0,184.0,75.0),
      TELEFALLINGCONE(24.0,-102.0,36.0,-105.0,162.0,59.0),
      AUTOPICKUP(-5.0,9.0,-5.0,9.0,35.0,25.0),
      CARRY(-21.0,145.0,-21.0,145.0,67.0,25.0),
      CARRYINTERMEDIATE(-21,90,-21,90,35.0,25.0),
      FEEDSTATION(70, -134, 70, -130, 67, 184),
      AUTOSCOREHIGH(94.0,40.0,109.0,54.0,161.0,23.0);
      //67, -137
      

      public final double shoulderCone;
      public final double shoulderCube;
      public final double elbowCone;
      public final double elbowCube;
      public final double wristCone;
      public final double wristCube;

      ArmPosition(double shoulderCone, double elbowCone, double shoulderCube, double elbowCube, double wristCone, double wristCube) {
        this.shoulderCone = shoulderCone;
        this.shoulderCube = shoulderCube;
        this.elbowCone = elbowCone;
        this.elbowCube = elbowCube;
        this.wristCone = wristCone;
        this.wristCube = wristCube;
      }
    }
  
    // Practice Arm
    /* 
    public enum ArmPosition {
    
      SCOREHIGH(110.0,54.0,110.0,70.0,165.0,23.0),
      SCOREMID(78.0,129.0,110.0,90.0,165.0,23.0),
      SCORELOW(100.0,145.0,100.0,145.0,185.0,75.0),
      SUBSTATION(66.0,-48.0,66.0,-61.0,255.0,114.0),
      AUTOPICKUP(-5.0,9.0,-5.0,9.0,35.0,25.0),
      TELESTANDINGCONE(35.0,-72.0,35.0,-72.0,194.0,75.0),
      TELEFALLINGCONE(29.0,-76.0,29.0,-76.0,130.0,75.0),
      CARRY(-21.0,155.0,-21.0,155.0,35.0,25.0),
      CARRYINTERMEDIATE(-21,90,-21,90,35.0,25.0);
      

      public final double shoulderCone;
      public final double shoulderCube;
      public final double elbowCone;
      public final double elbowCube;
      public final double wristCone;
      public final double wristCube;

      ArmPosition(double shoulderCone, double elbowCone, double shoulderCube, double elbowCube, double wristCone, double wristCube) {
        this.shoulderCone = shoulderCone;
        this.shoulderCube = shoulderCube;
        this.elbowCone = elbowCone;
        this.elbowCube = elbowCube;
        this.wristCone = wristCone;
        this.wristCube = wristCube;
      }
    }
 */
  }
/**
 * Based on a 75:1 Gear Ratio
 */
  public static final double TICKS_PER_DEGREES = 426.66667;

  public static final double ELBOW_OFFSET_FOR_PREMADE_SETPOINTS_IN_TICKS = (TICKS_PER_DEGREES * 90.0);//-54.0);
  public static final double SHOULDER_OFFSET_FOR_PREMADE_SETPOINTS_IN_TICKS = (TICKS_PER_DEGREES * 90.0);//115.0);

  public static enum GamePiece {
    Cone,
    Cube,
    Carry,
    CarryObject,
    Nothing;
  }

  public static enum AllianceColor {
    Red,
    Blue,
    UNKNOWN;

    public static AllianceColor getAllianceColor() {
      // If we're blue, return blue. Otherwise default to red (if red or invalid).
      return DriverStation.getAlliance() == Alliance.Blue ? Blue : Red;
    }
    public static AllianceColor getOpposingColor() {
      // The inverse of getAllianceColor
      return DriverStation.getAlliance() == Alliance.Blue ? Red : Blue;
    }
  }

}
