package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointConeNoWait;
import frc.robot.commands.auto.common.DriveToPickupCone1;
import frc.robot.commands.auto.common.ScoreConeMid;
import frc.robot.commands.auto.drive.AutoCartesianVectorProfileToPointTargetCommand;
import frc.robot.commands.auto.drive.AutoEngagePP2PBack;
import frc.robot.commands.swerve.Lockdown;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class blueStartLeftyLeftScoreO1GToppyBalance extends SequentialCommandGroup{
    public blueStartLeftyLeftScoreO1GToppyBalance(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ScoreConeMid(elbow, intake, intakespinner, robotContainer , shoulder),
            new DriveToPickupCone1(
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
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new ArmAutoSetpointConeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY),  
                new AutoCartesianVectorProfileToPointTargetCommand(
                    Constants.Auto.blueIntermediateMobilityPoint, 
                    drivetrain::getTranslation, 
                    drivetrain::velocity,
                    Constants.Auto.trajectoryCutoff,
                    3.0, 
                    Units.inchesToMeters(162),
                    Units.inchesToMeters(30), 
                    autoDrive, 
                    drivetrain,
                    heading
                )
            ),
            new WaitCommand(1),
            new AutoEngagePP2PBack(
                Constants.Auto.blueLeftCenterOfChargeStation, 
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
            ),
            new Lockdown(autoDrive, drivetrain, heading)
        );
    }        
}
