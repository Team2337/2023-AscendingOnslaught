package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.common.DriveToPickupCone1;
import frc.robot.commands.auto.common.ScoreConeMid;
import frc.robot.commands.auto.drive.AutoEngagePP2PBack;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class redStartMiddleRightScoreO6GMidBalance extends SequentialCommandGroup{
    public redStartMiddleRightScoreO6GMidBalance(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ScoreConeMid(elbow, intake, intakespinner, robotContainer , shoulder),
            new DriveToPickupCone1(
                Constants.Auto.redMiddleStagingMark, 
                autoDrive, 
                drivetrain, 
                elbow, 
                heading, 
                intake, 
                intakespinner, 
                robotContainer, 
                shoulder
            ),
            new WaitCommand(1),
            new AutoEngagePP2PBack(
                Constants.Auto.redRightCenterOfChargeStation, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(45), 
                Units.inchesToMeters(45), 
                drivetrain::getGyroscopePitch,
                autoDrive, 
                drivetrain,
                heading
                )
        );
    }        
}
