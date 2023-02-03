package frc.robot.commands.auto.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.CartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class BlueConstructTeleopAutoCommand extends SequentialCommandGroup {
    Supplier<Translation2d> waypoint2_inner = null;
    AutoDrive autoDrive;
    Drivetrain drivetrain;
    Heading heading;

    public BlueConstructTeleopAutoCommand(Supplier<Translation2d> waypoint2_inner,  AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {   
            this.waypoint2_inner = waypoint2_inner;
            this.autoDrive = autoDrive;
            this.drivetrain = drivetrain;
            this.heading = heading;
                          
              // waypoint2 = inner waypoint (if it exists)
            if (waypoint2_inner.get().getX() != 0) {
                addCommands(
                 // waypoint1 = outer waypoint
                new CartesianVectorProfileToPointTargetCommand(
                    drivetrain::getWaypointOuter, 
                    drivetrain::getTranslation,
                    1.5,
                    Units.inchesToMeters(80),
                    autoDrive, 
                    heading
                ),
                  new CartesianVectorProfileToPointTargetCommand(
                    drivetrain::getWaypointInner, 
                    drivetrain::getTranslation,
                    1.5,
                    Units.inchesToMeters(80),
                    autoDrive, 
                    heading
                  ),
                  new CartesianVectorProfileToPointTargetCommand(
                    drivetrain::getWaypointGoal, 
                    drivetrain::getTranslation,
                    1.5,
                    Units.inchesToMeters(80),
                    autoDrive, 
                    heading
                  )
              );
            } else {
                addCommands(
                 // waypoint1 = outer waypoint
                new CartesianVectorProfileToPointTargetCommand(
                    drivetrain::getWaypointOuter, 
                    drivetrain::getTranslation,
                    1.5,
                    Units.inchesToMeters(80),
                    autoDrive, 
                    heading
                ),
                  new CartesianVectorProfileToPointTargetCommand(
                    drivetrain::getWaypointGoal, 
                    drivetrain::getTranslation,
                    1.5,
                    Units.inchesToMeters(80),
                    autoDrive, 
                    heading
                  )
              );
            }
                
          }
    }
