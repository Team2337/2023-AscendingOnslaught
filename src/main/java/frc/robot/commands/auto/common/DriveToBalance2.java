package frc.robot.commands.auto.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.drive.AutoCartesianVectorProfileToPointTargetCommand;
import frc.robot.commands.auto.drive.AutoEngagePP2P;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class DriveToBalance2 extends SequentialCommandGroup{
    public DriveToBalance2(Translation2d waypoint, Translation2d target, AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, Shoulder shoulder) {
        addCommands(
            new AutoCartesianVectorProfileToPointTargetCommand(
                waypoint, 
                drivetrain::getTranslation, 
                drivetrain::velocity,
                Constants.Auto.trajectoryCutoff,
                3.0, 
                Units.inchesToMeters(162),
                Units.inchesToMeters(30), 
                autoDrive, 
                drivetrain,
                heading
            ),
            new AutoEngagePP2P(
                target, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(60), 
                Units.inchesToMeters(60), 
                drivetrain::getGyroscopePitch,
                autoDrive, 
                drivetrain,
                heading
            )
        );
    }
}
