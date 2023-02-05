package frc.robot.commands.auto.teleop;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.CartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class BlueConstructTeleopAutoCommand extends SequentialCommandGroup {
  AutoDrive autoDrive;
  Drivetrain drivetrain;
  Heading heading;
  RobotContainer robotContainer;
  private double trajectoryCutoff;

  public BlueConstructTeleopAutoCommand(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading, RobotContainer robotContainer) {
    this.autoDrive = autoDrive;
    this.drivetrain = drivetrain;
    this.heading = heading;
    this.robotContainer = robotContainer;
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
            Units.inchesToMeters(80),
            autoDrive,
            drivetrain,
            heading,
            robotContainer),
        new CartesianVectorProfileToPointTargetCommand(
            drivetrain::getWaypointInner,
            drivetrain::getTranslation,
            drivetrain::velocity,
            trajectoryCutoff,
            1.5,
            Units.inchesToMeters(80),
            autoDrive,
            drivetrain,
            heading,
            robotContainer),
        new CartesianVectorProfileToPointTargetCommand(
            drivetrain::getWaypointGoal,
            drivetrain::getTranslation,
            drivetrain::velocity,
            2,
            1.5,
            Units.inchesToMeters(80),
            autoDrive,
            drivetrain,
            heading,
            robotContainer));

  }

}
