package frc.robot.commands.auto.pathplanner.red.balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.common.RedBalanceFront;
import frc.robot.commands.auto.common.RedGBotScoreC8GMidScoreO8;
import frc.robot.commands.auto.common.RedRightyMidGBotScoreC8GMidScoreO8;
import frc.robot.commands.auto.test.FollowTrajectoryCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class redScoreLow extends SequentialCommandGroup{
    public redScoreLow(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new IntakeReverseCube(intake).withTimeout(0.2),
            new FollowTrajectoryCommand(robotContainer.redSendIt, true, drivetrain::getPose, autoDrive, drivetrain, heading)
            );
    }
}
