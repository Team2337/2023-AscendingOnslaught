package frc.robot.commands.auto.common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointShoulderCube;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class ScoreCubeMid extends SequentialCommandGroup{
    public ScoreCubeMid(Elbow elbow, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ArmAutoSetpointShoulderCube(Constants.Arm.ArmPosition.AUTOSCOREMID, 15, elbow, shoulder, intakespinner, robotContainer),
            new ArmAutoSetpointCubeWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.AUTOSCOREMID).withTimeout(0.5),
            new IntakeReverseCube(intake).withTimeout(0.5)
        );
    }
}
