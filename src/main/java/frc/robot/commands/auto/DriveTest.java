package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.CartesianProfiledPointToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class DriveTest extends SequentialCommandGroup{
    Drivetrain drivetrain;
    public DriveTest(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, Shoulder shoulder) {
        this.drivetrain = drivetrain;
        addCommands(
            new WaitCommand(2),
            new CartesianProfiledPointToPointCommand(
                new Translation2d(2, 0), 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3.0, 
                3.0,
                Units.inchesToMeters(15), 
                Units.inchesToMeters(15), 
                autoDrive, 
                drivetrain, 
                heading),
            new WaitCommand(2),
            new CartesianProfiledPointToPointCommand(
                new Translation2d(0, 0), 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3.0, 
                3.0,
                Units.inchesToMeters(15), 
                Units.inchesToMeters(15), 
                autoDrive, 
                drivetrain, 
                heading)
            // new WaitCommand(2),
            // new DriveToPickupCone1(Constants.Auto.redMiddleStagingMark, autoDrive, drivetrain, elbow, heading, intake, intakespinner, shoulder),
            // new WaitCommand(2),
            // new AutoCartesianVectorProfileToPointTargetCommand(
            //     Constants.Auto.redGridRightRobotRight, 
            //     drivetrain::getTranslation, 
            //     drivetrain::velocity,
            //     Constants.Auto.trajectoryTolerance,
            //     3.5, 
            //     Units.inchesToMeters(60),
            //     Units.inchesToMeters(15), 
            //     autoDrive, 
            //     drivetrain,
            //     heading
            // ),
            // new ArmAutoSetpointWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)
        );
    }        
}
