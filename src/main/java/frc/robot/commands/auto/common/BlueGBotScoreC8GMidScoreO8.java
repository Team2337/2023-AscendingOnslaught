package frc.robot.commands.auto.common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeWait;
import frc.robot.commands.auto.aboveChassis.IntakeReverseAuto;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.test.FollowTrajectoryCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class BlueGBotScoreC8GMidScoreO8 extends SequentialCommandGroup {
    
    public BlueGBotScoreC8GMidScoreO8(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {

        addCommands(
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(robotContainer.blueRightyMidGBottom, true, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.NEWAUTOPICKUPBLUEBUMP).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.NEWAUTOPICKUP)),
                new WaitCommand(0.25).andThen(new IntakeReverseAuto(intake).withTimeout(3))
            ),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(robotContainer.blueScoreC8, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH))
            ),
            new IntakeReverseCube(intake).withTimeout(0.2),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(robotContainer.blueGMid2, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.NEWAUTOPICKUPBLUEBUMP).withTimeout(1.0).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.NEWAUTOPICKUP)),
                new IntakeReverseAuto(intake).withTimeout(3.5)
            ),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(robotContainer.blueScoreO8, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID))
            ),
            new IntakeReverseCube(intake).withTimeout(0.2)
    );

    }
}
