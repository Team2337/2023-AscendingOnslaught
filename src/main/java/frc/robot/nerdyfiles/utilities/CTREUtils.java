package frc.robot.nerdyfiles.utilities;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class CTREUtils {

  /**
   * Creates a default limit configuration. This is base the current limit
   * configuration we use on our motors to prevent overheating the motors
   * in catastrophic scenerios.
   */
  public static StatorCurrentLimitConfiguration defaultCurrentLimit() {
    return new StatorCurrentLimitConfiguration(true, 50.0, 40.0, 3.0);
  }

  public static StatorCurrentLimitConfiguration lowCurrentLimit() {
    return new StatorCurrentLimitConfiguration(true, 30.0, 30.0, 1.0);
  }

  public static StatorCurrentLimitConfiguration wristCurrentLimit() {
    return new StatorCurrentLimitConfiguration(true, 35.0, 35.0, 1.0);
  }

}
