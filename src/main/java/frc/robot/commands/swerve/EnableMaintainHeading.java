package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Heading;

public class EnableMaintainHeading extends InstantCommand{
  private Heading heading;

  public EnableMaintainHeading(Heading heading) {
    this.heading = heading;
    addRequirements(heading); 
  }

  @Override
  public void initialize() {
    heading.enableMaintainHeading();
  }
}
