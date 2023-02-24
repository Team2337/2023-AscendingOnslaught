package frc.robot.commands.auto.common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointConeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointConeWait;
import frc.robot.commands.auto.aboveChassis.IntakeReverseAuto;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class ScoreConeHigh extends SequentialCommandGroup{
    public ScoreConeHigh(Elbow elbow, Intake intake, IntakeSpinnerLamprey intakespinner, Shoulder shoulder) {
        addCommands(
            new ArmAutoSetpointConeWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.SCOREHIGH).withTimeout(1.5),
            new IntakeReverseAuto(intake).withTimeout(0.75),
            new ArmAutoSetpointConeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)
        );
    }
}
