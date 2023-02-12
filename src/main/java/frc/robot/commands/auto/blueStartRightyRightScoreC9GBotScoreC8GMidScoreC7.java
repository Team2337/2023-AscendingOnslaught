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

public class blueStartRightyRightScoreC9GBotScoreC8GMidScoreC7 extends SequentialCommandGroup{
    public blueStartRightyRightScoreC9GBotScoreC8GMidScoreC7(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, Shoulder shoulder) {
        addCommands(
            new ScoreConeHigh(elbow, intake, shoulder),
            new DriveToPickupCone1(Constants.Auto.blueBottomStagingMark, autoDrive, drivetrain, elbow, heading, intake, shoulder),
            new DriveToScoreHigh1(Constants.Auto.blueGridRightRobotCenter, Constants.Arm.SCOREHIGH.SHOULDER, Constants.Arm.SCOREHIGH.ELBOW, autoDrive, drivetrain, elbow, heading, intake, shoulder),
            new ScoreConeHigh(elbow, intake, shoulder),
            new DriveToPickupCone2(Constants.Auto.blueRightIntermediaryFar, Constants.Auto.blueMiddleStagingMark, autoDrive, drivetrain, elbow, heading, intake, shoulder),
            new DriveToScoreHigh3(Constants.Auto.blueRightIntermediaryFar, Constants.Auto.blueRightIntermediaryAutoNear, Constants.Auto.blueGridRightRobotLeft, Constants.Arm.SCOREHIGH.SHOULDER, Constants.Arm.SCOREHIGH.ELBOW, autoDrive, drivetrain, elbow, heading, intake, shoulder),
            new ScoreConeHigh(elbow, intake, shoulder)
        );
    }        
}