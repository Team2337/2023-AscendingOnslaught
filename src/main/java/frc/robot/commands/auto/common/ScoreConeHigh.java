package frc.robot.commands.auto.common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointConeWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointShoulderCone;
import frc.robot.commands.auto.aboveChassis.IntakeReverseAuto;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class ScoreConeHigh extends SequentialCommandGroup{
    public ScoreConeHigh(Elbow elbow, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ArmAutoSetpointShoulderCone(Constants.Arm.ArmPosition.AUTOSCOREHIGH, 15, elbow, shoulder, intakespinner, robotContainer),
            new ArmAutoSetpointConeWait(elbow, shoulder, intakespinner, robotContainer, Constants.Arm.ArmPosition.AUTOSCOREHIGH).withTimeout(0.75),
            new IntakeReverseAuto(intake).withTimeout(0.35)
        );
    }
}
