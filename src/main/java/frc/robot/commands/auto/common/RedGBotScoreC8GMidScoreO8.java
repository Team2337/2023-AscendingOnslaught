package frc.robot.commands.auto.common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeNoWait;
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

public class RedGBotScoreC8GMidScoreO8 extends SequentialCommandGroup {

    public RedGBotScoreC8GMidScoreO8(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(robotContainer.redRightyRightGBottom, true, drivetrain::getPose, autoDrive, drivetrain, heading),
                        new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRISTYOSHIRED).withTimeout(2.0).andThen(new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRISTYOSHIRED)),
                        new IntakeReverseAuto(intake).withTimeout(2.45)),
                new ParallelCommandGroup(
                        new ArmAutoSetpointCubeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRIST2YOSHIRED),
                        new IntakeReverseAuto(intake).withTimeout(0.5)
                ),
                new ParallelCommandGroup(
                        new IntakeReverseAuto(intake).withTimeout(0.5),
                        new FollowTrajectoryCommand(robotContainer.redScoreC8Yeet, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                        new WaitCommand(0.5).andThen(new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH))),
                new IntakeReverseCube(intake).withTimeout(0.2),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(robotContainer.redGMidYeet, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                        new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRISTYOSHIRED).withTimeout(2.0).andThen(new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRISTYOSHIRED)),
                        new IntakeReverseAuto(intake).withTimeout(2.8)
                ),
                new ParallelCommandGroup(
                        new ArmAutoSetpointCubeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRIST2YOSHIRED),
                        new IntakeReverseAuto(intake).withTimeout(0.5)
                ),
                new ParallelCommandGroup(
                        new IntakeReverseAuto(intake).withTimeout(0.5),
                        new FollowTrajectoryCommand(robotContainer.redScoreO8Yeet, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                        new WaitCommand(0.5).andThen(new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID))),
                new IntakeReverseCube(intake).withTimeout(0.2));
    }
}