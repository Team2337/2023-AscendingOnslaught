package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmSetpointCommand;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointConeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointElbowCube;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointShoulderCube;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.common.DriveToPickupCube1;
import frc.robot.commands.auto.common.DriveToPickupCube1Yoshi;
import frc.robot.commands.auto.common.DriveToPickupCube2;
import frc.robot.commands.auto.common.DriveToPickupCube2Yoshi;
import frc.robot.commands.auto.common.DriveToScoreCube2HighYoshi;
import frc.robot.commands.auto.common.DriveToScoreMid1Cube;
import frc.robot.commands.auto.common.DriveToScoreMid1CubeYoshi;
import frc.robot.commands.auto.common.ScoreConeMid;
import frc.robot.commands.auto.drive.AutoEngagePP2PFront;
import frc.robot.commands.swerve.Lockdown;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class blueStartLeftyLeftScoreW1GToppyScoreO2GTopScoreC2BalanceYoshi extends SequentialCommandGroup{
    public blueStartLeftyLeftScoreW1GToppyScoreO2GTopScoreC2BalanceYoshi(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new IntakeReverseCube(intake).withTimeout(0.4),
            new DriveToPickupCube1Yoshi(
                Constants.Auto.blueToppyTopStagingMarkYoshi, 
                autoDrive, 
                drivetrain, 
                elbow, 
                heading, 
                intake, 
                intakespinner, 
                robotContainer, 
                shoulder
            ),
            new DriveToScoreMid1CubeYoshi(
                Constants.Auto.blueGridLeftRobotCenter, 
                Constants.Arm.ArmPosition.CARRY, 
                autoDrive, 
                drivetrain, 
                elbow, 
                heading, 
                intake, 
                intakespinner, 
                shoulder
            ),
            new IntakeReverseCube(intake).withTimeout(0.4),
            new DriveToPickupCube2Yoshi(
                Constants.Auto.blueLeftIntermediaryFarYoshi, 
                Constants.Auto.blueTopStagingMarkYoshi, 
                autoDrive, 
                drivetrain, 
                elbow, 
                heading, 
                intake, 
                intakespinner, 
                robotContainer, 
                shoulder
            ),
            new DriveToScoreCube2HighYoshi(
                Constants.Auto.blueLeftIntermediaryFar, 
                Constants.Auto.blueGridLeftRobotCenterYoshi, 
                Constants.Arm.ArmPosition.CARRY,
                autoDrive, 
                drivetrain, 
                elbow, 
                heading, 
                intake, 
                intakespinner, 
                shoulder
            ),
            new IntakeReverseCube(intake).withTimeout(0.5)
            // new ParallelCommandGroup(
            //     new SequentialCommandGroup(
            //         new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
            //         new ArmAutoSetpointConeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
            //     ),
            //     new AutoEngagePP2PFront(
            //         Constants.Auto.blueRightCenterOfChargeStation, 
            //         drivetrain::getTranslation, 
            //         drivetrain::getRotation, 
            //         3, 
            //         3, 
            //         Units.inchesToMeters(45), 
            //         Units.inchesToMeters(45), 
            //         drivetrain::getGyroscopePitch,
            //         autoDrive, 
            //         drivetrain,
            //         heading
            //     )
            // ),
            // new Lockdown(autoDrive, drivetrain, heading).withTimeout(0.25)
        );
    }        
}
