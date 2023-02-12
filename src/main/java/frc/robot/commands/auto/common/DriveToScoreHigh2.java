package frc.robot.commands.auto.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointNoWait;
import frc.robot.commands.auto.drive.AutoCartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class DriveToScoreHigh2 extends ParallelCommandGroup{
    public DriveToScoreHigh2(Translation2d waypoint, Translation2d target, double shoulderSetpoint, double elbowSetpoint, AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, Shoulder shoulder) {
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
            new ParallelCommandGroup(
                new AutoCartesianVectorProfileToPointTargetCommand(
                    target, 
                    drivetrain::getTranslation, 
                    drivetrain::velocity,
                    Constants.Auto.trajectoryTolerance,
                    3.0, 
                    Units.inchesToMeters(162),
                    Units.inchesToMeters(30), 
                    autoDrive, 
                    drivetrain,
                    heading
                ),
                new ArmAutoSetpointNoWait(elbow, shoulder, shoulderSetpoint, elbowSetpoint)
        ));
    }
}
