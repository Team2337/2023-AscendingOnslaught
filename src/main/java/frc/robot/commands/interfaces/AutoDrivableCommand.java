package frc.robot.commands.interfaces;

import frc.robot.subsystems.AutoDrive;

public interface AutoDrivableCommand {
  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented);
}
