package frc.robot.commands.auto.common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class RedRightyMidGBotScoreC8GMidScoreO8 extends SequentialCommandGroup {
    public RedRightyMidGBotScoreC8GMidScoreO8(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
                new ParallelCommandGroup(
                        // new FollowTrajectoryCommand(robotContainer.redRightyMidGBottom, true, drivetrain::getPose, autoDrive, drivetrain, heading),
                        new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRISTYOSHI),
                        new IntakeReverseAuto(intake).withTimeout(2.45)),
                new ParallelCommandGroup(
                        // new FollowTrajectoryCommand(robotContainer.redScoreC8, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                        new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH)),
                new IntakeReverseCube(intake).withTimeout(0.2),
                new ParallelCommandGroup(
                        // new FollowTrajectoryCommand(robotContainer.redAvoidChargeStation, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                        new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRIST3YOSHIRED), // 3
                        new IntakeReverseAuto(intake).withTimeout(2.8)),
                new ParallelCommandGroup(
                        // new FollowTrajectoryCommand(robotContainer.redScoreO8, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                        new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID)),
                new IntakeReverseCube(intake).withTimeout(0.2)
        );
    }
}