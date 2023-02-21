package frc.robot.commands.auto.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointConeWait;
import frc.robot.commands.auto.aboveChassis.IntakeForwardAuto;
import frc.robot.commands.auto.drive.AutoCartesianVectorProfileToPointTargetCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class DriveToPickupCone1 extends ParallelCommandGroup{
    public DriveToPickupCone1(Translation2d target, AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, Shoulder shoulder){
        addCommands(
            new AutoCartesianVectorProfileToPointTargetCommand(
                target, 
                drivetrain::getTranslation, 
                drivetrain::velocity,
                Constants.Auto.trajectoryTolerance,
                3, 
                Units.inchesToMeters(80),
                Units.inchesToMeters(15), 
                autoDrive, 
                drivetrain,
                heading
                ),
            new ArmAutoSetpointConeWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.TELESTANDINGCONE).withTimeout(5),
            new WaitCommand(0.75).andThen(new IntakeForwardAuto(intake))
        );
    }
}
