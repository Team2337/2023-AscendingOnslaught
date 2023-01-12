package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Rumble extends CommandBase{
  private XboxController controller;

  public Rumble(XboxController controller) {
    this.controller = controller;
  }

  public void initialize() {
    // Rumble the joystick if a ball has triggered the intake sensor. Takes a value of 0 to 1 as a percentage of 65535.
    controller.setRumble(RumbleType.kLeftRumble, 1);
    controller.setRumble(RumbleType.kRightRumble, 1);
  }

  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kLeftRumble, 0);
    controller.setRumble(RumbleType.kRightRumble, 0);
  }
}