package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.common.DriveToPickupCone1;
import frc.robot.commands.auto.common.DriveToPickupCone2;
import frc.robot.commands.auto.common.DriveToScoreHigh1;
import frc.robot.commands.auto.common.DriveToScoreHigh3;
import frc.robot.commands.auto.common.ScoreConeHigh;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class blueStartLeftyLeftScoreC1GToppyScoreC2GTopScoreC3 extends SequentialCommandGroup{
    public blueStartLeftyLeftScoreC1GToppyScoreC2GTopScoreC3(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, Shoulder shoulder) {
        addCommands(
            new ScoreConeHigh(elbow, intake, shoulder),
            new DriveToPickupCone1(Constants.Auto.blueToppyTopStagingMark, autoDrive, drivetrain, elbow, heading, intake, shoulder),
            new DriveToScoreHigh1(Constants.Auto.blueGridLeftRobotCenter, Constants.Arm.SCOREHIGH.SHOULDER, Constants.Arm.SCOREHIGH.ELBOW, autoDrive, drivetrain, elbow, heading, intake, shoulder),
            new ScoreConeHigh(elbow, intake, shoulder),
            new DriveToPickupCone2(Constants.Auto.blueLeftIntermediaryFar, Constants.Auto.blueTopStagingMark, autoDrive, drivetrain, elbow, heading, intake, shoulder),
            new DriveToScoreHigh3(Constants.Auto.blueLeftIntermediaryFar, Constants.Auto.blueLeftIntermediaryNear, Constants.Auto.blueGridLeftRobotRight, Constants.Arm.SCOREHIGH.SHOULDER, Constants.Arm.SCOREHIGH.ELBOW, autoDrive, drivetrain, elbow, heading, intake, shoulder),
            new ScoreConeHigh(elbow, intake, shoulder)
        );
    }        
}