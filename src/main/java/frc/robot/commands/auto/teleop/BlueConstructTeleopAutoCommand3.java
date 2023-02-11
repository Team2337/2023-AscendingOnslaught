package frc.robot.commands.auto.teleop;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.drive.CartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class BlueConstructTeleopAutoCommand3 extends SequentialCommandGroup {
  AutoDrive autoDrive;
  Drivetrain drivetrain;
  Heading heading;
  private double trajectoryCutoff;

  public BlueConstructTeleopAutoCommand3(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    this.autoDrive = autoDrive;
    this.drivetrain = drivetrain;
    this.heading = heading;
    trajectoryCutoff = Constants.Auto.trajectoryCutoff;

    // waypoint2 = inner waypoint (if it exists)
    addCommands(
        // waypoint1 = outer waypoint
        new CartesianVectorProfileToPointTargetCommand(
            drivetrain::getWaypointOuter,
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
            drivetrain::getWaypointInner,
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
