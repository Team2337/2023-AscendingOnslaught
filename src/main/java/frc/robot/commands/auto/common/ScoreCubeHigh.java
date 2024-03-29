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

public class ScoreCubeHigh extends SequentialCommandGroup{
    public ScoreCubeHigh(Elbow elbow, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ArmAutoSetpointShoulderCube(Constants.Arm.ArmPosition.SCOREHIGH, 15, elbow, shoulder, intakespinner, robotContainer),
            new ArmAutoSetpointCubeWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH).withTimeout(2),
            new IntakeReverseCube(intake).withTimeout(0.5)
        );
    }
}
