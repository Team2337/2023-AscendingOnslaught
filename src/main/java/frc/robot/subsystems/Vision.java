package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AllianceColor;
import frc.robot.Constants.DriverDashboardPositions;
import frc.robot.nerdyfiles.vision.LimelightUtilities;

public class Vision extends SubsystemBase {

  private enum LimelightKey {
    Pose("botpose"),
    Pipeline("pipeline"),
    LEDMode("ledMode"),
    X("tx"),
    Y("ty"),
    LATENCY("tl"),
    VALID_TARGET("tv"),
    STREAM("stream");


    private String key;

    private LimelightKey(String key) {
      this.key = key;
    }
  }

  public enum LimelightColor {
    BLUE,
    ORANGE,
    PINK
  }

  public static enum Pipeline {
    DEFAULT(0);

    private int number;

    private Pipeline(int number) {
      this.number = number;
    }

    public static Pipeline withNumber(int number) {
      for (Pipeline pipeline : Pipeline.values()) {
        if (pipeline.number == number) {
          return pipeline;
        }
      }
      return null;
    }
  }

  public static enum LEDMode {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private int value;

    private LEDMode(int value) {
      this.value = value;
    }

    public static LEDMode withValue(int value) {
      for (LEDMode mode : LEDMode.values()) {
        if (mode.value == value) {
          return mode;
        }
      }
      return null;
    }
  }

  // Only fetch our LL values once per periodic cycle
  private Pipeline currentPipelineB, currentPipelineO, currentPipelineP;
  private LEDMode currentLEDModeB, currentLEDModeO, currentLEDModeP;
  private double txB, txO, txP = 0.0;
  private double tyB, tyO, tyP = 0.0;
  private double[] tposeB, tposeO, tposeP = {0, 0, 0, 0, 0, 0};
  private double[] defaultPose = {0, 0, 0, 0, 0, 0};
  private double latencyB, latencyO, latencyP = 0.0;
  private boolean hasValidTargetB, hasValidTargetO, hasValidTargetP = false;
  private double distanceToTargetMetersB = 0.0;
  private double distanceToTargetMetersO = 0.0;
  private double distanceToTargetMetersP = 0.0;
  private LimelightColor color;
  private String allianceColor;
  private String botPoseColor = "botpose_wpi";
  RobotContainer robotContainer;
  private String pipelineAllianceColor;

  public int relocalizeCounter = 0;

  public Vision(RobotContainer robotContainer) {
    // Automatically switch our Limelight to our default pipeline on construction
    switchPipeLine(Pipeline.DEFAULT, LimelightColor.BLUE);
    switchPipeLine(Pipeline.DEFAULT, LimelightColor.ORANGE);
    switchPipeLine(Pipeline.DEFAULT, LimelightColor.PINK);

    if (AllianceColor.getAllianceColor() == AllianceColor.Blue) {
      pipelineAllianceColor = "blue";
    } else {
      pipelineAllianceColor = "red";
    }
    //TODO: Fix this  
    // Systems check
    if (Constants.DO_SYSTEMS_CHECK) {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;

      // systemsCheck.addBoolean("Limelight Connected", () -> (latency > 0))
      //   .withPosition(SystemsCheckPositions.LIMELIGHT.x, SystemsCheckPositions.LIMELIGHT.y)
      //   .withSize(3, 3);
    }
      ShuffleboardTab driverDashboard = Constants.DRIVER_DASHBOARD;

      driverDashboard.addNumber("Distance to Target (inches)", () -> Units.metersToInches(distanceToTargetMetersB))
      .withPosition(DriverDashboardPositions.DISTANCE_TO_TARGET.x, DriverDashboardPositions.DISTANCE_TO_TARGET.y)
      .withSize(3, 3);
    }

  @Override
  public void periodic() {
    botPoseColor = "botpose_wpi";
    if (Robot.allianceColor.equals("Blue")) {
      allianceColor = "blue";
    } else {
      allianceColor = "red";
    }
    botPoseColor = botPoseColor + allianceColor;
    SmartDashboard.putString("Bot Pose Color", botPoseColor);
    SmartDashboard.putString("Pipeline Alliance Color", pipelineAllianceColor);

    tposeB = NetworkTableInstance.getDefault().getTable("limelight-blue").getEntry(botPoseColor).getDoubleArray(defaultPose);
    currentPipelineB = Pipeline.withNumber(getIntValue(LimelightKey.Pipeline, LimelightColor.BLUE));
    currentLEDModeB = LEDMode.withValue(getIntValue(LimelightKey.LEDMode, LimelightColor.BLUE));
    txB = getDoubleValue(LimelightKey.X, LimelightColor.BLUE);
    tyB = getDoubleValue(LimelightKey.Y, LimelightColor.BLUE);
    latencyB = getDoubleValue(LimelightKey.LATENCY, LimelightColor.BLUE);
    hasValidTargetB = getDoubleValue(LimelightKey.VALID_TARGET, LimelightColor.BLUE) == 1.0;

    distanceToTargetMetersB = 0.0;
    if (hasValidTargetB) {
      distanceToTargetMetersB = calculateDistanceToTargetMeters(LimelightColor.BLUE);
    }

    tposeO = NetworkTableInstance.getDefault().getTable("limelight-orange").getEntry(botPoseColor).getDoubleArray(defaultPose);
    currentPipelineO = Pipeline.withNumber(getIntValue(LimelightKey.Pipeline, LimelightColor.ORANGE));
    currentLEDModeO = LEDMode.withValue(getIntValue(LimelightKey.LEDMode, LimelightColor.ORANGE));
    txO = getDoubleValue(LimelightKey.X, LimelightColor.ORANGE);
    tyO = getDoubleValue(LimelightKey.Y, LimelightColor.ORANGE);
    latencyO = getDoubleValue(LimelightKey.LATENCY, LimelightColor.ORANGE);
    hasValidTargetO = getDoubleValue(LimelightKey.VALID_TARGET, LimelightColor.ORANGE) == 1.0;

    distanceToTargetMetersO = 0.0;
    if (hasValidTargetO) {
      distanceToTargetMetersO = calculateDistanceToTargetMeters(LimelightColor.ORANGE);
    }

    tposeP = NetworkTableInstance.getDefault().getTable("limelight-pink").getEntry(botPoseColor).getDoubleArray(defaultPose);
    currentPipelineP = Pipeline.withNumber(getIntValue(LimelightKey.Pipeline, LimelightColor.PINK));
    currentLEDModeP = LEDMode.withValue(getIntValue(LimelightKey.LEDMode, LimelightColor.PINK));
    txP = getDoubleValue(LimelightKey.X, LimelightColor.PINK);
    tyP = getDoubleValue(LimelightKey.Y, LimelightColor.PINK);
    latencyP = getDoubleValue(LimelightKey.LATENCY, LimelightColor.PINK);
    hasValidTargetP = getDoubleValue(LimelightKey.VALID_TARGET, LimelightColor.PINK) == 1.0;

    distanceToTargetMetersP = 0.0;
    if (hasValidTargetP) {
      distanceToTargetMetersP = calculateDistanceToTargetMeters(LimelightColor.PINK);
    }


    log();
  }

  private void log() {
    if (Constants.DashboardLogging.VISION) {
      SmartDashboard.putNumber("Vision/# of relocalization", relocalizeCounter);
      SmartDashboard.putNumber("Vision/Blue Vision Pose X", getVisionPoseX(LimelightColor.BLUE));
      SmartDashboard.putNumber("Vision/Blue Vision Pose Y", getVisionPoseY(LimelightColor.BLUE));
      SmartDashboard.putNumber("Vision/latency blue", getLatency(LimelightColor.BLUE));
      SmartDashboard.putBoolean("Vision/Valid Target blue", hasActiveTarget(LimelightColor.BLUE));
      SmartDashboard.putNumber("Vision/Orange Vision Pose X", getVisionPoseX(LimelightColor.ORANGE));
      SmartDashboard.putNumber("Vision/Orange Vision Pose Y", getVisionPoseY(LimelightColor.ORANGE));
      SmartDashboard.putNumber("Vision/latency orange", getLatency(LimelightColor.ORANGE));
      SmartDashboard.putBoolean("Vision/Valid Target orange", hasActiveTarget(LimelightColor.ORANGE));
      SmartDashboard.putNumber("Vision/Pink Vision Pose X", getVisionPoseX(LimelightColor.PINK));
      SmartDashboard.putNumber("Vision/Pink Vision Pose Y", getVisionPoseY(LimelightColor.PINK));
      SmartDashboard.putNumber("Vision/Latency Pink", getLatency(LimelightColor.PINK));
      SmartDashboard.putBoolean("Vision/Valid Target Pink", hasActiveTarget(LimelightColor.PINK));
      SmartDashboard.putNumber("Vision/Blue Distance To Target (inches)", Units.metersToInches(distanceToTargetMetersB));
      SmartDashboard.putNumber("Vision/Orange Distance To Target (inches)", Units.metersToInches(distanceToTargetMetersO));
      SmartDashboard.putNumber("Vision/Pink Distance To Target (inches)", Units.metersToInches(distanceToTargetMetersP));
    //  SmartDashboard.putNumber("Vision/Robot Pose X", Units.metersToInches(getVisionPoseX()));
    //SmartDashboard.putNumber("Vision/Robot Pose Y", Units.metersToInches(getVisionPoseY()));
    }
  }

  /** Limelight Network Table Access */

  private static NetworkTableEntry getLimelightEntryOrange(LimelightKey key) {
    return NetworkTableInstance.getDefault().getTable("limelight-orange").getEntry(key.key);
  }

  private static NetworkTableEntry getLimelightEntryBlue(LimelightKey key) {
    return NetworkTableInstance.getDefault().getTable("limelight-blue").getEntry(key.key);
  }

  private static NetworkTableEntry getLimelightEntryPink(LimelightKey key) {
    return NetworkTableInstance.getDefault().getTable("limelight-pink").getEntry(key.key);
  }

  /**
   * Get the double value for a key from the Limelight NetworkTables.
   *
   * @param key - The Limelight NetworkTables key
   * @return - the double value for the key in the Limelight NetworkTables. 0.0 if
   *         the key does not exist.
   */
  private static double getDoubleValue(LimelightKey key, LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return getLimelightEntryBlue(key).getDouble(0);
    } else if (color == LimelightColor.PINK) {
      return getLimelightEntryPink(key).getDouble(0);
    } else {
      return getLimelightEntryOrange(key).getDouble(0);
    }
  }


  /**
   * Get the int value for a key from the Limelight NetworkTables.
   *
   * @param key - The Limelight NetworkTables key
   * @return - the int value for the key in the Limelight NetworkTables. 0 if
   *         the key does not exist.
   */
  private static int getIntValue(LimelightKey key, LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return getLimelightEntryBlue(key).getNumber(0).intValue();
    }  else if (color == LimelightColor.PINK) {
      return getLimelightEntryPink(key).getNumber(0).intValue();
    } else {
      return getLimelightEntryOrange(key).getNumber(0).intValue();
    }
  }

  /**
   * Sets the Limelight entry value.
   *
   * @param value the value to set
   * @return False if the entry exists with a different type
   */
  private static boolean setValue(LimelightKey key, LimelightColor color, Number value) {
    if (color == LimelightColor.BLUE) {
      return getLimelightEntryBlue(key).setNumber(value);
    }  else if (color == LimelightColor.PINK) {
      return getLimelightEntryPink(key).setNumber(value);
    } else {
      return getLimelightEntryOrange(key).setNumber(value);
    }
  }

  /** Limelight API */

  /**
   * Sets the LED mode to on, off, or blink
   */
  public void setLEDMode(LEDMode mode, LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      if (currentLEDModeB != mode) {
        if (setValue(LimelightKey.LEDMode, LimelightColor.BLUE, mode.value)) {
          currentLEDModeB = mode;
        }
      }
    }  else if (color == LimelightColor.PINK) {
      if (currentLEDModeP != mode) {
        currentLEDModeP = mode;
      }
    } else {
      if (currentLEDModeO != mode) {
        if (setValue(LimelightKey.LEDMode, LimelightColor.ORANGE, mode.value)) {
          currentLEDModeO = mode;
        }
      }
    }
  }

  /**
   * Gets the Limelight LED mode
   */
  public LEDMode getLEDMode(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return currentLEDModeB;
    } else if (color == LimelightColor.PINK) {
      return currentLEDModeP;
    } else {
      return currentLEDModeO;
    }
  }

  /**
   * Sets the pipeline of the Limelight
   */
  public void switchPipeLine(Pipeline pipeline, LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      if (currentPipelineB != pipeline) {
        if (setValue(LimelightKey.Pipeline, LimelightColor.BLUE, pipeline.number)) {
          currentPipelineB = pipeline;
        }
      }
    } else if (color == LimelightColor.PINK) {
      if (currentPipelineP != pipeline) {
        if (setValue(LimelightKey.Pipeline, LimelightColor.PINK, pipeline.number)) {
          currentPipelineP = pipeline;
        }
      }
    } else {
      if (currentPipelineO != pipeline) {
        if (setValue(LimelightKey.Pipeline, LimelightColor.ORANGE, pipeline.number)) {
          currentPipelineO = pipeline;
        }
      }
    }
  }

  /**
   * Gets the current pipeline on the Limelight
   * @return - Double value Limelight pipeline (0 -> 9)
   */
  public Pipeline getPipeline(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return currentPipelineB;
    } else if (color == LimelightColor.PINK) {
      return currentPipelineP;
    } else {
      return currentPipelineO;
    }
  }

  /**
   * Returns the horizontal offset from the crosshair to the target in degree (-27 to 27) 
   * @return - tx value from the limelight plus an offset if desired to allow for fine tuning of vision centering or if we want to shoot to the side of the target
   */
  //TODO: Check vision offset
  public double getTx(LimelightColor color) { 
    if (color == LimelightColor.BLUE) {
      return txB + Constants.VISION_OFFSET;
    } else if (color == LimelightColor.PINK) {
      return txP + Constants.VISION_OFFSET;
    } else {
      return txO + Constants.VISION_OFFSET;
    }
  }

  /**
   * Returns the Vertical offset from the crosshair to the target in degrees (-20.5 to 20.5)
   * @return ty value from the limelight
   */
  public double getTy(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return tyB;
    } else if (color == LimelightColor.PINK) {
      return tyP;
    } else {
      return tyO;
    }
  }

  public double[] getVisionPose(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return tposeB;
    } else if (color == LimelightColor.PINK) {
      return tposeP;
    } else {
      return tposeO;
    }
  }

  public double getVisionPoseX(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return tposeB[0];
    } else if (color == LimelightColor.PINK) {
      return tposeP[0];
    } else {
      return tposeO[0];
    }
  }

  public double getVisionPoseY(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return tposeB[1];
    } else if (color == LimelightColor.PINK) {
      return tposeP[1];
    } else {
      return tposeO[1];
    }
  }

  public double getVisionRotation(LimelightColor color) { 
    if (color == LimelightColor.BLUE) {
      return tposeB[5];
    } else if (color == LimelightColor.PINK) {
      return tposeP[5];
    } else {
      return tposeO[5];
    }
  }

  public double getLatency(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return latencyB;
    } else if (color == LimelightColor.PINK) {
      return latencyP;
    } else {
      return latencyO;
    }
  }

  public boolean hasActiveTarget(LimelightColor color) { 
    if (color == LimelightColor.BLUE) {
      return hasValidTargetB;
    } else if (color == LimelightColor.PINK) {
      return hasValidTargetP;
    } else {
      return hasValidTargetO;
    }
  }

  public boolean isOnTarget(LimelightColor color) { 
    if (color == LimelightColor.BLUE) {
      return Math.abs(txB) < Constants.VISION_TOLERANCE;
    } else if (color == LimelightColor.PINK) {
      return Math.abs(txP) < Constants.VISION_TOLERANCE;
    } else {
      return Math.abs(txO) < Constants.VISION_TOLERANCE;
    }
  }

  public double getDistanceToCenterHubMeters() {
    return distanceToTargetMetersB + Constants.Vision.VISION_TARGET_OFFSET_FROM_HUB_CENTER_METERS;
  }

  public void incrementRelocalizeCounter() {
    relocalizeCounter++;
  }

  private double calculateDistanceToTargetMeters(LimelightColor color) { 
    return LimelightUtilities.calculateDistanceToTargetMeters(
      Constants.getInstance().LIMELIGHT_CAMERA_HEIGHT_METERS,
      Constants.HUB_HEIGHT_METERS,
      Constants.getInstance().LIMEILGHT_CAMERA_ANGLE,
      Rotation2d.fromDegrees(getTy(color)),
      Rotation2d.fromDegrees(getTx(color))
    );
  }

  public double calculateDistanceToTargetInches() {
    return Units.metersToInches(distanceToTargetMetersB);
  }

}
