package frc.robot.commands.auto.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeNoWait;
import frc.robot.commands.auto.drive.AutoCartesianVectorNoCutoff;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class DriveToScoreMid1CubeYoshi extends ParallelCommandGroup{
    public DriveToScoreMid1CubeYoshi(Translation2d target, ArmPosition armPosition, AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, Shoulder shoulder) {
        addCommands(
            new AutoCartesianVectorNoCutoff(
                target, 
                drivetrain::getTranslation, 
                drivetrain::velocity,
                Constants.Auto.trajectoryTolerance,
                3.0, 
                Units.inchesToMeters(200),
                Units.inchesToMeters(100), 
                autoDrive, 
                drivetrain,
                heading
            ).withTimeout(3),
            new ArmAutoSetpointCubeNoWait(elbow, shoulder, intakespinner, armPosition)
        );
    }
}
