package frc.robot.commands.auto.teleop;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.drive.CartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Vision;

public class RedConstructTeleopAutoCommand3 extends SequentialCommandGroup {
  AutoDrive autoDrive;
  Drivetrain drivetrain;
  Heading heading;
  Vision vision;
  private double trajectoryCutoff;

  public RedConstructTeleopAutoCommand3(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading, Vision vision) {
    this.autoDrive = autoDrive;
    this.drivetrain = drivetrain;
    this.heading = heading;
    this.vision = vision;
    trajectoryCutoff = Constants.Auto.trajectoryCutoff;

    addRequirements(vision);

    // waypoint2 = inner waypoint (if it exists)
    addCommands(
        // waypoint1 = outer waypoint
        new CartesianVectorProfileToPointTargetCommand(
            drivetrain::getWaypointOuter,
            drivetrain::getTranslation,
            drivetrain::velocity,
            trajectoryCutoff,
            1.5,
            Units.inchesToMeters(60),
            Units.inchesToMeters(15),
            autoDrive,
            drivetrain,
            heading),
        new CartesianVectorProfileToPointTargetCommand(
            drivetrain::getWaypointInner,
            drivetrain::getTranslation,
            drivetrain::velocity,
            trajectoryCutoff,
            1.5,
            Units.inchesToMeters(60),
            Units.inchesToMeters(15),
            autoDrive,
            drivetrain,
            heading),
        new CartesianVectorProfileToPointTargetCommand(
            drivetrain::getWaypointGoal,
            drivetrain::getTranslation,
            drivetrain::velocity,
            Constants.Auto.trajectoryTolerance,
            1.5,
            Units.inchesToMeters(60),
            Units.inchesToMeters(15),
            autoDrive,
            drivetrain,
            heading));
  }

}
