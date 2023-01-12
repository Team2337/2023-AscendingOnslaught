package frc.robot.nerdyfiles.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * Code for operator station.
 *
 * Copied from 2020 code
 * @see https://github.com/Team2337/2020-Perpetual-Supercharger/blob/master/src/main/java/frc/robot/nerdyfiles/controller/NerdyOperatorStation.java
 */
public class NerdyOperatorStation extends Joystick {

  public NerdyOperatorStation(int port) {
    super(port);
  }

  /*
    * --- DRIVER STATION CONTROLS ---
    *
    * These button/switch names and ports will need to be updated each year if the
    * driver's station is redesigned.
    *
    */
  public JoystickButton  greenButton  = new JoystickButton(this, 1);
  public JoystickButton  redButton    = new JoystickButton(this, 2);
  public JoystickButton  whiteButton  = new JoystickButton(this, 7);
  public JoystickButton  yellowButton = new JoystickButton(this, 8);
  public JoystickButton  blueButton   = new JoystickButton(this, 9);
  public JoystickButton  blackButton  = new JoystickButton(this, 10);

  public JoystickButton  clearSwitch  = new JoystickButton(this, 3);
  public JoystickButton  yellowSwitch = new JoystickButton(this, 4);
  public JoystickButton  blueSwitch   = new JoystickButton(this, 5);
  public JoystickButton  blackSwitch  = new JoystickButton(this, 6);

  public JoystickButton redLeftSwitch = new JoystickButton(this, 11);
  public JoystickButton redRightSwitch = new JoystickButton(this, 12);

  public boolean isClearSwitchOn() {
    return this.clearSwitch.getAsBoolean();
  }

  public boolean isYellowSwitchOn() {
    return this.yellowSwitch.getAsBoolean();
  }

  public boolean isBlackSwitchOn() {
    return this.blackSwitch.getAsBoolean();
  }

  public boolean isBlueSwitchOn() {
    return this.blueSwitch.getAsBoolean();
  }

  public boolean isRedLeftSwitchOn() {
    return this.redLeftSwitch.getAsBoolean();
  }

  public boolean isRedRightSwitchOn() {
    return this.redRightSwitch.getAsBoolean();
  }
}
