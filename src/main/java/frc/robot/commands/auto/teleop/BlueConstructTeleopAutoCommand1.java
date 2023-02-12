package frc.robot.commands.auto.teleop;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.drive.CartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class BlueConstructTeleopAutoCommand1 extends SequentialCommandGroup {
  AutoDrive autoDrive;
  Drivetrain drivetrain;
  Heading heading;

  public BlueConstructTeleopAutoCommand1(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    this.autoDrive = autoDrive;
    this.drivetrain = drivetrain;
    this.heading = heading;

    addCommands(
        new CartesianVectorProfileToPointTargetCommand(
            drivetrain::getWaypointGoal,
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
