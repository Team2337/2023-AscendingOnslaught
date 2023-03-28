package frc.robot.commands.auto.pathplanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.common.RedBalanceFront;
import frc.robot.commands.auto.common.RedGBotScoreC8GMidScoreO8;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class redRightyRightScoreW9GBotScoreC8GMidScoreO8Balance extends SequentialCommandGroup{
    public redRightyRightScoreW9GBotScoreC8GMidScoreO8Balance(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new IntakeReverseCube(intake).withTimeout(0.2),
            new RedGBotScoreC8GMidScoreO8(autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer,shoulder),

        //     new ParallelCommandGroup(
        //         new FollowTrajectoryCommand(robotContainer.redRightyRightGBottom, true, drivetrain::getPose, autoDrive, drivetrain, heading),
        //         new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRISTYOSHI),
        //         new IntakeReverseAuto(intake).withTimeout(2.25)
        //     ),
        //     new ParallelCommandGroup(
        //         new FollowTrajectoryCommand(robotContainer.redScoreC8, false, drivetrain::getPose, autoDrive, drivetrain, heading),
        //         new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH)
        //     ),
        // new IntakeReverseCube(intake).withTimeout(0.2),
        // new ParallelCommandGroup(
        //     new FollowTrajectoryCommand(robotContainer.redAvoidChargeStation, false, drivetrain::getPose, autoDrive, drivetrain, heading),
        //     new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRIST3YOSHI),//3
        //     new IntakeReverseAuto(intake).withTimeout(2.8)
        // ),
        // new ParallelCommandGroup(
        //     new FollowTrajectoryCommand(robotContainer.redScoreO8, false, drivetrain::getPose, autoDrive, drivetrain, heading),
        //     new ArmAutoSetpointCubeNoWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH)
        // ),
        // new IntakeReverseCube(intake).withTimeout(0.2),
    //     new SequentialCommandGroup(
    //         new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
    //         new ArmAutoSetpointCubeNoWait(0.8, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
    //     ),



    new RedBalanceFront(autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer,shoulder)

        // new ArmAutoShoulderConeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.AUTOCARRYINTERMEDIATE),
        // new ParallelRaceGroup(
        //     new FollowTrajectoryCommand(robotContainer.redChargeStation, false, drivetrain::getPose, autoDrive, drivetrain, heading),
        //     new AutoBalanceIndicatorFront(drivetrain::getGyroscopePitch)
        // )
        //new FollowTrajectoryCommand(robotContainer.redLockdown, true, drivetrain::getPose, autoDrive, drivetrain, heading).withTimeout(0.15)
     );
    }
}
