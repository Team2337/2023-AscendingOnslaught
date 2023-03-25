package frc.robot.commands.auto.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeWait;
import frc.robot.commands.auto.aboveChassis.IntakeReverseAuto;
import frc.robot.commands.auto.drive.AutoCartesianVectorNoCutoff;
import frc.robot.commands.auto.drive.AutoCartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class DriveToPickupCube2Yoshi extends SequentialCommandGroup{
    public DriveToPickupCube2Yoshi(Translation2d waypoint, Translation2d target, AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ParallelCommandGroup(
                new AutoCartesianVectorProfileToPointTargetCommand(
                    waypoint, 
                    drivetrain::getTranslation, 
                    drivetrain::velocity,
                    Constants.Auto.trajectoryCutoffYoshi,
                    2.0, 
                    Units.inchesToMeters(200),
                    Units.inchesToMeters(100), 
                    autoDrive, 
                    drivetrain,
                    heading
                ), 
                new ArmAutoSetpointCubeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRIST3YOSHI)
            ),   
            new ParallelCommandGroup(
                new AutoCartesianVectorNoCutoff(
                    target, 
                    drivetrain::getTranslation, 
                    drivetrain::velocity,
                    Constants.Auto.trajectoryTolerance,
                    2.0, 
                    Units.inchesToMeters(200),
                    Units.inchesToMeters(100), 
                    autoDrive, 
                    drivetrain,
                    heading
                ),
                new ArmAutoSetpointCubeWait(0.9, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.FLOORPICKUPWRIST3YOSHI).withTimeout(1.5),
                new IntakeReverseAuto(intake).withTimeout(1.5)
                // new InstantCommand(() -> robotContainer.setPastArmPositionFallenCone())
            )     
        );
    }
}
