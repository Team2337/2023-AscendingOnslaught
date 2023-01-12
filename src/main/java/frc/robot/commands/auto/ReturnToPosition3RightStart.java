package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class ReturnToPosition3RightStart extends SequentialCommandGroup {

  public ReturnToPosition3RightStart(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    addCommands(
      new ProfiledPointToPointCommand(Constants.Auto.kPosition3RightStart, drivetrain::getTranslation, 0.5, 0.025, Units.inchesToMeters(60), 4, autoDrive, heading),
      new WaitCommand(1)
    );
  }

}