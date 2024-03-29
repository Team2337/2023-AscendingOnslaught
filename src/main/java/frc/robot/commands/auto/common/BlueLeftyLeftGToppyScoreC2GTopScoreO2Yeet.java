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

public class BlueLeftyLeftGToppyScoreC2GTopScoreO2Yeet extends SequentialCommandGroup{
    public BlueLeftyLeftGToppyScoreC2GTopScoreO2Yeet(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(robotContainer.blueLeftyLeftGToppy, true, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.NEWAUTOPICKUP),
                new IntakeReverseAuto(intake).withTimeout(2.7)
            ),
            new ParallelCommandGroup(
                // new FollowTrajectoryCommand(robotContainer.blueScoreC2Yeet, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH)
            ),
            new IntakeReverseCube(intake).withTimeout(0.2),
            new ParallelCommandGroup(
                // new FollowTrajectoryCommand(robotContainer.blueGTopYeet, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.NEWAUTOPICKUP),
                new IntakeReverseAuto(intake).withTimeout(2.7)
            ),
            new ParallelCommandGroup(
                // new FollowTrajectoryCommand(robotContainer.blueScoreO2Yeet, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID)
            ),
            new IntakeReverseCube(intake).withTimeout(0.2)
        );
    }
}
