package frc.robot.commands.auto.pathplanner.red.bump;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.common.RedGToppyScoreC2GTopScoreO2;
import frc.robot.commands.auto.common.ScoreConeHigh;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class redLeftyMidScoreC1GToppyScoreC2GTopScoreO2 extends SequentialCommandGroup{
    public redLeftyMidScoreC1GToppyScoreC2GTopScoreO2(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ScoreConeHigh(elbow, intake, intakespinner, robotContainer, shoulder),
            new RedGToppyScoreC2GTopScoreO2(autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer,shoulder),

            // new ParallelCommandGroup(
            //     new FollowTrajectoryCommand(robotContainer.redLeftyLeftGToppy, true, drivetrain::getPose, autoDrive, drivetrain, heading),
            //     new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPYOSHI).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPYOSHIBUMP)),
            //     new WaitCommand(0.25).andThen(new IntakeReverseAuto(intake).withTimeout(3))
            // ),
            // new ParallelCommandGroup(
            //     new FollowTrajectoryCommand(robotContainer.redScoreC2, false, drivetrain::getPose, autoDrive, drivetrain, heading),
            //     new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH))
            // ),
            // new IntakeReverseCube(intake).withTimeout(0.2),
            // new ParallelCommandGroup(
            //     new FollowTrajectoryCommand(robotContainer.redGTop, false, drivetrain::getPose, autoDrive, drivetrain, heading),
            //     new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPYOSHI).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUP3YOSHIBUMP)),
            //     new IntakeReverseAuto(intake).withTimeout(3.0)
            // ),
            // new ParallelCommandGroup(
            //     new FollowTrajectoryCommand(robotContainer.redScoreO2, false, drivetrain::getPose, autoDrive, drivetrain, heading),
            //     new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID).withTimeout(1.5).andThen(new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID))
            // ),
            // new IntakeReverseCube(intake).withTimeout(0.2),
            new SequentialCommandGroup(
                new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
                new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
            )
        );
    }
}
