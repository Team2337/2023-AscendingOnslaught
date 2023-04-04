package frc.robot.commands.auto.pathplanner.blue.yeet;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.common.BlueLeftyMidGToppyScoreC2GTopScoreO2;
import frc.robot.commands.auto.test.FollowTrajectoryCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class blueStartLeftyMidScoreW2GToppyScoreC2GTopScoreO2SendIt extends SequentialCommandGroup{
    public blueStartLeftyMidScoreW2GToppyScoreC2GTopScoreO2SendIt(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new IntakeReverseCube(intake).withTimeout(0.2),
            new BlueLeftyMidGToppyScoreC2GTopScoreO2(autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer, shoulder),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
                    new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
                ),
                new FollowTrajectoryCommand(robotContainer.blueSendIt, false, drivetrain::getPose, autoDrive, drivetrain, heading)
            )
        );    
    }
}
