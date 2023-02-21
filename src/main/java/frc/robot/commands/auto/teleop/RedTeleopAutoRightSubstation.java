package frc.robot.commands.auto.teleop;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.drive.CartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class RedTeleopAutoRightSubstation extends SequentialCommandGroup {
  AutoDrive autoDrive;
  Drivetrain drivetrain;
  Heading heading;
  double trajectoryCutoff;

  public RedTeleopAutoRightSubstation(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    this.autoDrive = autoDrive;
    this.drivetrain = drivetrain;
    this.heading = heading;
    this.trajectoryCutoff = Constants.Auto.trajectoryCutoff;

    addCommands(
      new CartesianVectorProfileToPointTargetCommand(
        () -> Constants.Auto.redSubstationIntermediary,
        drivetrain::getTranslation,
        drivetrain::velocity,
        trajectoryCutoff,
        1.5,
        Units.inchesToMeters(162),
        Units.inchesToMeters(80),
        autoDrive,
        drivetrain,
        heading),
     new CartesianVectorProfileToPointTargetCommand(
        () -> Constants.Auto.redRightSubstationPickup,
        drivetrain::getTranslation,
        drivetrain::velocity,
        Constants.Auto.trajectoryTolerance,
        1.5,
        Units.inchesToMeters(162),
        Units.inchesToMeters(80),
        autoDrive,
        drivetrain,
        heading));

  }

}
