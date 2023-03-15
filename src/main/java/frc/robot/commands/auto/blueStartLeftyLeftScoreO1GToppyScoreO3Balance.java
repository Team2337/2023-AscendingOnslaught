package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointConeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.common.DriveToPickupCube1;
import frc.robot.commands.auto.common.DriveToScoreMid1Cube;
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

public class blueStartLeftyLeftScoreO1GToppyScoreO3Balance extends SequentialCommandGroup{
    public blueStartLeftyLeftScoreO1GToppyScoreO3Balance(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
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
                Constants.Arm.ArmPosition.AUTOSCOREMID, 
                autoDrive, 
                drivetrain, 
                elbow, 
                heading, 
                intake, 
                intakespinner, 
                shoulder
            ),
            new IntakeReverseCube(intake).withTimeout(0.4),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
                    new ArmAutoSetpointConeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
                ),
                new AutoEngagePP2PFront(
                    Constants.Auto.blueRightCenterOfChargeStation, 
                    drivetrain::getTranslation, 
                    drivetrain::getRotation, 
                    3, 
                    3, 
                    Units.inchesToMeters(45), 
                    Units.inchesToMeters(45), 
                    drivetrain::getGyroscopePitch,
                    autoDrive, 
                    drivetrain,
                    heading
                )
            ),
            new Lockdown(autoDrive, drivetrain, heading).withTimeout(0.25)
        );
    }        
}
