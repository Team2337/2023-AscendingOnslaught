package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.common.DriveToPickupCone1;
import frc.robot.commands.auto.common.DriveToPickupCone2;
import frc.robot.commands.auto.common.DriveToScoreHigh1;
import frc.robot.commands.auto.common.ScoreConeHigh;
import frc.robot.commands.auto.common.ScoreConeMid;
import frc.robot.commands.auto.drive.CartesianProfiledPointToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class redLeftyLeftScoreC1GToppyScoreO1GTopScoreC2 extends SequentialCommandGroup{
    public redLeftyLeftScoreC1GToppyScoreO1GTopScoreC2(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer,Shoulder shoulder) {
        addCommands(
            new ScoreConeHigh(elbow, intake, intakespinner, robotContainer, shoulder),
            new DriveToPickupCone2(Constants.Auto.redLeftIntermediaryFar, Constants.Auto.redToppyTopStagingMark, autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer, shoulder),
            new DriveToScoreHigh1(Constants.Auto.redGridLeftRobotLeft, Constants.Arm.ArmPosition.CARRYINTERMEDIATE, autoDrive, drivetrain, elbow, heading, intake, intakespinner, shoulder),
            new ScoreConeMid(elbow, intake, intakespinner, robotContainer, shoulder)
        );
    }        
}
