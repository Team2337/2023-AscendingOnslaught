package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointConeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.common.DriveToPickupCone2;
import frc.robot.commands.auto.common.DriveToPickupCube1;
import frc.robot.commands.auto.common.DriveToPickupCube2;
import frc.robot.commands.auto.common.DriveToScoreHigh1;
import frc.robot.commands.auto.common.DriveToScoreMid1Cube;
import frc.robot.commands.auto.common.ScoreConeMid;
import frc.robot.commands.auto.common.ScoreCubeHigh;
import frc.robot.commands.auto.common.ScoreCubeMid;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class blueStartLeftyLeftScoreO1GToppyScoreO2GTop extends SequentialCommandGroup{
    public blueStartLeftyLeftScoreO1GToppyScoreO2GTop(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ScoreConeMid(elbow, intake, intakespinner, robotContainer , shoulder),
            new DriveToPickupCube1(
                Constants.Auto.blueToppyTopStagingMark, 
                autoDrive, 
                drivetrain, 
                elbow, 
                heading, 
                intake, 
                intakespinner, 
                robotContainer, 
                shoulder
            ),
            new WaitCommand(0.25),
            new DriveToScoreMid1Cube(
                Constants.Auto.blueGridLeftRobotCenter, 
                Constants.Arm.ArmPosition.SCOREMID, 
                autoDrive, 
                drivetrain, 
                elbow, 
                heading, 
                intake, 
                intakespinner, 
                shoulder
            ),
            new IntakeReverseCube(intake).withTimeout(0.4),
            new DriveToPickupCube2(
                Constants.Auto.blueLeftIntermediaryFar, 
                Constants.Auto.blueTopStagingMark, 
                autoDrive, 
                drivetrain, 
                elbow, 
                heading, 
                intake, 
                intakespinner, 
                robotContainer, 
                shoulder
            )
            // new ArmAutoSetpointWithEndingCube(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
            // new ArmAutoSetpointCubeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
        );
    }        
}
