package frc.robot.commands.auto.pathplanner.blue.balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.common.ScoreConeHigh;
import frc.robot.commands.auto.test.FollowTrajectoryCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class blueMiddleBalance extends SequentialCommandGroup{
    public blueMiddleBalance(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ScoreConeHigh(elbow, intake, intakespinner, robotContainer, shoulder),
                new SequentialCommandGroup(
                    new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
                    new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
                ),
            new FollowTrajectoryCommand(robotContainer.blueMidBalance, true, drivetrain::getPose, autoDrive, drivetrain, heading),
            new FollowTrajectoryCommand(robotContainer.blueLockdown, true, drivetrain::getPose, autoDrive, drivetrain, heading).withTimeout(0.15)
            );
    }
}
