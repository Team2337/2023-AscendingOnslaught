package frc.robot.commands.auto.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeWait;
import frc.robot.commands.auto.aboveChassis.IntakeReverseAuto;
import frc.robot.commands.auto.drive.AutoCartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class DriveToPickupCube1Slow extends ParallelCommandGroup{
    public DriveToPickupCube1Slow(Translation2d target, AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder){
        addCommands(
            new AutoCartesianVectorProfileToPointTargetCommand(
                target, 
                drivetrain::getTranslation, 
                drivetrain::velocity,
                Constants.Auto.trajectoryTolerance,
                3, 
                Units.inchesToMeters(160),
                Units.inchesToMeters(60), 
                autoDrive, 
                drivetrain,
                heading
                ),
            new InstantCommand(() -> robotContainer.setGamePiece(GamePiece.Cube)),
            new WaitCommand(3.0).andThen(new ArmAutoSetpointCubeWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.AUTOTELEFALLINGCONE)).withTimeout(6),
            new WaitCommand(3.0).andThen(new IntakeReverseAuto(intake)).withTimeout(6.5)
        );
    }
}