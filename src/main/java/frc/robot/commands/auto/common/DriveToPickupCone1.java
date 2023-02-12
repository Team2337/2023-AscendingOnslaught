package frc.robot.commands.auto.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWait;
import frc.robot.commands.auto.aboveChassis.IntakeForwardAuto;
import frc.robot.commands.auto.drive.AutoCartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class DriveToPickupCone1 extends ParallelCommandGroup{
    public DriveToPickupCone1(Translation2d target, AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, Shoulder shoulder) {
        addCommands(
            new AutoCartesianVectorProfileToPointTargetCommand(
                target, 
                drivetrain::getTranslation, 
                drivetrain::velocity,
                Constants.Auto.trajectoryTolerance,
                3.0, 
                Units.inchesToMeters(162),
                Units.inchesToMeters(60), 
                autoDrive, 
                drivetrain,
                heading
                ),
            new WaitCommand(1).andThen(new ArmAutoSetpointWait(elbow, shoulder, Constants.Arm.AUTOPICKUP.SHOULDER, Constants.Arm.AUTOPICKUP.ELBOW).withTimeout(2)),
            new WaitCommand(0.75).andThen(new IntakeForwardAuto(intake).withTimeout(1.5))
        );
    }
}
