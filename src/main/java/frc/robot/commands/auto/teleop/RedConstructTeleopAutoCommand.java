package frc.robot.commands.auto.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.CartesianVectorProfileToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class RedConstructTeleopAutoCommand extends SequentialCommandGroup {
    Translation2d waypoint1_outer = null;
    Translation2d waypoint2_inner = null;
    Translation2d waypoint3_goal = null;
    Supplier<Integer> teleopAutoPosition;
    AutoDrive autoDrive;
    Drivetrain drivetrain;
    Heading heading;

    public RedConstructTeleopAutoCommand(Supplier<Integer> teleopAutoPosition, AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {   
            this.teleopAutoPosition = teleopAutoPosition;
            this.autoDrive = autoDrive;
            this.drivetrain = drivetrain;
            this.heading = heading;
      
                    switch (teleopAutoPosition.get()) {
                      case 1:
                        waypoint1_outer = Constants.Auto.redRightIntermediaryFar;
                        waypoint3_goal = Constants.Auto.red1;
                        break;
                      case 2:
                        waypoint1_outer = Constants.Auto.redRightIntermediaryFar;
                        waypoint3_goal = Constants.Auto.red2;
                        break;
                      case 3:
                        waypoint1_outer = Constants.Auto.redRightIntermediaryFar;
                        waypoint2_inner = Constants.Auto.redRightIntermediaryNear;
                        waypoint3_goal = Constants.Auto.red3;
                        break;
                      case 4:
                        waypoint1_outer = Constants.Auto.redRightIntermediaryFar;
                        waypoint2_inner = Constants.Auto.redRightIntermediaryNear;
                        waypoint3_goal = Constants.Auto.red4;
                        break;
                      case 5:
                        waypoint1_outer = Constants.Auto.redLeftIntermediaryFar;
                        waypoint2_inner = Constants.Auto.redLeftIntermediaryNear;
                        waypoint3_goal = Constants.Auto.red5;
                        break;
                      case 6:
                        waypoint1_outer = Constants.Auto.redLeftIntermediaryFar;
                        waypoint2_inner = Constants.Auto.redLeftIntermediaryNear;
                        waypoint3_goal = Constants.Auto.red6;
                        break;
                      case 7:
                        waypoint1_outer = Constants.Auto.redLeftIntermediaryFar;
                        waypoint2_inner = Constants.Auto.redLeftIntermediaryNear;
                        waypoint3_goal = Constants.Auto.red7;
                        break;
                      case 8:
                        waypoint1_outer = Constants.Auto.redLeftIntermediaryFar;
                        waypoint3_goal = Constants.Auto.red8;
                        break;
                      case 9:
                        waypoint1_outer = Constants.Auto.redLeftIntermediaryFar;
                        waypoint3_goal = Constants.Auto.red9;
                        break;
                      case 10:
                        //waypoint1 = Constants.Auto.red;
                        waypoint3_goal = Constants.Auto.red10;
                        break;
                      case 11:
                        //waypoint1 = Constants.Auto.redLeftIntermediaryFar;
                        waypoint3_goal = Constants.Auto.red11;
                        break;
                    }

              // waypoint2 = inner waypoint (if it exists)
            if (waypoint2_inner != null) {
                addCommands(
                 // waypoint1 = outer waypoint
                new CartesianVectorProfileToPointCommand(
                    waypoint1_outer, 
                    drivetrain::getTranslation,
                    1.5,
                    Units.inchesToMeters(80),
                    autoDrive, 
                    heading
                ),
                  new CartesianVectorProfileToPointCommand(
                    waypoint2_inner, 
                    drivetrain::getTranslation,
                    1.5,
                    Units.inchesToMeters(80),
                    autoDrive, 
                    heading
                  ),
                  new CartesianVectorProfileToPointCommand(
                    waypoint3_goal, 
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
                new CartesianVectorProfileToPointCommand(
                    waypoint1_outer, 
                    drivetrain::getTranslation,
                    1.5,
                    Units.inchesToMeters(80),
                    autoDrive, 
                    heading
                ),
                  new CartesianVectorProfileToPointCommand(
                    waypoint3_goal, 
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
