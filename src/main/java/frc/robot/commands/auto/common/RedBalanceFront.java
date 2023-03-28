package frc.robot.commands.auto.common;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoShoulderConeNoWait;
import frc.robot.commands.auto.drive.AutoBalanceIndicatorFront;
import frc.robot.commands.auto.test.FollowTrajectoryCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class RedBalanceFront extends SequentialCommandGroup {

    public RedBalanceFront(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
    
    addCommands(
    new ArmAutoShoulderConeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.AUTOCARRYINTERMEDIATE),
    new ParallelRaceGroup(
        new FollowTrajectoryCommand(robotContainer.redChargeStation, false, drivetrain::getPose, autoDrive, drivetrain, heading),
        new AutoBalanceIndicatorFront(drivetrain::getGyroscopePitch)
    ),
    new FollowTrajectoryCommand(robotContainer.redLockdown, true, drivetrain::getPose, autoDrive, drivetrain, heading).withTimeout(0.15),
    new ArmAutoSetpointCubeNoWait(0.8, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)
 );


}
}
