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

}
