package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class LimelightHeadingAndInstantRelocalizeCommand extends SequentialCommandGroup {

  public LimelightHeadingAndInstantRelocalizeCommand(Drivetrain drivetrain, Heading heading, Vision vision) {
    addCommands(
      new LimeLightHeadingCommand(drivetrain, heading, vision).withTimeout(2.0),
      new InstantRelocalizeCommand(drivetrain, vision)
    );
  }
}
