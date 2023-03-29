package frc.robot.commands.auto.pathplanner;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.aboveChassis.ArmAutoShoulderConeNoWait;
import frc.robot.commands.auto.aboveChassis.IntakeReverseAuto;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.common.BlueBalanceFront;
import frc.robot.commands.auto.drive.AutoBalanceIndicatorFront;
import frc.robot.commands.auto.test.FollowTrajectoryCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class blueStartLeftyLeftScoreW1GToppyScoreC2GTopScoreO2Balance extends SequentialCommandGroup{
    public blueStartLeftyLeftScoreW1GToppyScoreC2GTopScoreO2Balance(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new IntakeReverseCube(intake).withTimeout(0.2),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(robotContainer.blueLeftyLeftGToppy, true, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRISTYOSHI),
                new IntakeReverseAuto(intake).withTimeout(2.25)
            ),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(robotContainer.blueScoreC2, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH)
            ),
            new IntakeReverseCube(intake).withTimeout(0.2),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(robotContainer.AvoidChargeStation, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRIST3YOSHIBLUE),
                new IntakeReverseAuto(intake).withTimeout(2.5)
            ),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(robotContainer.blueScoreO2, false, drivetrain::getPose, autoDrive, drivetrain, heading),
                new ArmAutoSetpointCubeNoWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREMID)
            ),
            new IntakeReverseCube(intake).withTimeout(0.2),
            // // new SequentialCommandGroup(
            // //     new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
            // //     new ArmAutoSetpointCubeNoWait(0.8, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
            // // ),
            // // new ArmAutoShoulderConeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.AUTOCARRYINTERMEDIATE),
            // new ParallelRaceGroup(
            //     new FollowTrajectoryCommand(robotContainer.blueChargeStation2, false, drivetrain::getPose, autoDrive, drivetrain, heading),
            //     new AutoBalanceIndicatorFront(drivetrain::getGyroscopePitch)
            // ),
            // new FollowTrajectoryCommand(robotContainer.blueLockdown, true, drivetrain::getPose, autoDrive, drivetrain, heading).withTimeout(0.15) 
            new BlueBalanceFront(autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer,shoulder)  
        );    
    }
}
