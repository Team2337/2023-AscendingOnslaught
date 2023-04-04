package frc.robot.commands.auto.pathplanner.red.bump;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.common.RedGToppyScoreC2GTopScoreO2;
import frc.robot.commands.auto.test.FollowTrajectoryCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class redLeftyLeftScoreW1GToppyScoreC2GTopScoreO2SendIt extends SequentialCommandGroup{
    public redLeftyLeftScoreW1GToppyScoreC2GTopScoreO2SendIt(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new IntakeReverseCube(intake).withTimeout(0.2),
            new RedGToppyScoreC2GTopScoreO2(autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer,shoulder)

            // new ParallelCommandGroup(
            //     new FollowTrajectoryCommand(robotContainer.redLeftyLeftGToppyFast, true, drivetrain::getPose, autoDrive, drivetrain, heading),
            //     new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPYOSHI).withTimeout(2.0).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUP2YOSHIBUMP)),
            //     new WaitCommand(0.25).andThen(new IntakeReverseAuto(intake).withTimeout(3.0))
            // ),
            // new ParallelCommandGroup(
            //     new FollowTrajectoryCommand(robotContainer.redScoreC2Fast, false, drivetrain::getPose, autoDrive, drivetrain, heading),
            //     new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH))
            // ),
            // new IntakeReverseCube(intake).withTimeout(0.2),
            // new ParallelCommandGroup(
            //     new FollowTrajectoryCommand(robotContainer.redGTopFast, false, drivetrain::getPose, autoDrive, drivetrain, heading),
            //     new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPYOSHI).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUP2YOSHIBUMP)),
            //     new IntakeReverseAuto(intake).withTimeout(3.0)
            // ),
            // new ParallelCommandGroup(
            //     new FollowTrajectoryCommand(robotContainer.redScoreO2Fast, false, drivetrain::getPose, autoDrive, drivetrain, heading),
            //     new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID))
            // ),
            // new IntakeReverseCube(intake).withTimeout(0.2),
            // new ParallelCommandGroup(
            //     new SequentialCommandGroup(
            //         new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
            //         new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
            //     ),
            //     new FollowTrajectoryCommand(robotContainer.redLeftSendIt, false, drivetrain::getPose, autoDrive, drivetrain, heading)
            // )
        );
    }
}
