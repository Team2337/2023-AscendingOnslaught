package frc.robot.commands.auto.common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeWait;
import frc.robot.commands.auto.aboveChassis.IntakeReverseAuto;
import frc.robot.commands.auto.test.FollowTrajectoryCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class DriveToPickupCube2PathPlanner extends ParallelCommandGroup{
    public DriveToPickupCube2PathPlanner(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new FollowTrajectoryCommand(robotContainer.AvoidChargeStation,true, drivetrain::getPose, autoDrive, drivetrain, heading),
            new ArmAutoSetpointCubeWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.TELEFALLINGCONE).withTimeout(5),
            new WaitCommand(0.75).andThen(new IntakeReverseAuto(intake).withTimeout(4.25))
        );
    }
}
