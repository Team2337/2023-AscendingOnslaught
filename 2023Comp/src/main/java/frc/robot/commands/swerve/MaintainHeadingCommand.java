package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Heading;

public class MaintainHeadingCommand extends CommandBase{
  private Heading heading;
  private double rotationDegrees;

  public MaintainHeadingCommand(double rotationDegrees, Heading heading) {
    this.rotationDegrees = rotationDegrees;
    this.heading = heading;
    addRequirements(heading); 
  }

  @Override
  public void initialize() {
    heading.enableMaintainHeading();
    heading.setMaintainHeading(Rotation2d.fromDegrees(rotationDegrees));
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
