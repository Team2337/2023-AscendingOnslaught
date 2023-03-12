package frc.robot.commands.LED;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDState;
import frc.robot.nerdyfiles.leds.LED;

public class LEDRunnable extends CommandBase{
  private final RobotContainer robotContainer;
  private final LED led;
  private Supplier<Boolean> hasGamepiece;

  public LEDRunnable(LED led, RobotContainer robotContainer, Supplier<Boolean> hasGamepiece) {
    this.led = led;
    this.robotContainer = robotContainer;
    this.hasGamepiece = hasGamepiece;

    addRequirements(led);
  }
  @Override
  public void execute() {
    // if (robotContainer.getPDHChannelBlue() != 0) {
    //     led.setColor(Color.kBlue);
    // } else {
    //     led.setColor(Color.kRed);
    // }
    led.setFrontColor(Color.kRed);
    if (DriverStation.isTeleopEnabled() && robotContainer.getLEDState() == LEDState.Cone) {
      led.setLeftColor(Color.kYellow);
      led.setRightColor(Color.kYellow);
    }
    if (DriverStation.isTeleopEnabled() && robotContainer.getLEDState() == LEDState.Cube) {
      led.setLeftColor(Color.kPurple);
      led.setRightColor(Color.kPurple);
    }
    if (DriverStation.isTeleopEnabled() && robotContainer.getLEDState() == LEDState.Nothing) {
      led.setLeftColor(Color.kBlack);
      led.setRightColor(Color.kBlack);
    }
    if (DriverStation.isTeleopEnabled() && robotContainer.getLEDState() == LEDState.HasGamePiece) {
      led.setLeftColor(Color.kRed);
      led.setRightColor(Color.kRed);
    }
    if (DriverStation.isDisabled()) {
      led.setLeftColor(Color.kRed);
      led.setRightColor(Color.kRed);
    }
    // if (DriverStation.isTeleop() && hasGamepiece.get() == true) {
    //   led.setLeftColor(Color.kRed);
    //   led.setRightColor(Color.kRed);
    // }
    
    // if (robotContainer.getYellowSwitchStatus() && robotContainer.getGyroscopeRoll() < Constants.CLIMBER_ROLL) {
    //   led.setColor(Color.kPurple);
    // } else if (robotContainer.isShooterUpToLEDSpeed() && robotContainer.hasActiveTarget()) {
    //   led.setColor(Color.kRed, robotContainer.getTx());
    // } else if (robotContainer.isShooterUpToLEDSpeed()) {
    //   led.setColor(Color.kBlue);
    // } else if (robotContainer.getOperatorStartStatus() || robotContainer.getOperatorBackStatus()) {
    //   led.setColorMiddle();
    // } else if (robotContainer.hasActiveTarget()) {
    //   led.setColor(Color.kYellow, robotContainer.getTx());
    // } else if (robotContainer.getOperatorRightTriggerStatus() && DriverStation.isTeleop()) {
    //   led.setColorMiddle();
    // } else {
    //   led.setOff();
    // }
  }
}