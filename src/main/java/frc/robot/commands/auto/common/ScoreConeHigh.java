package frc.robot.commands.auto.common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWait;
import frc.robot.commands.auto.aboveChassis.IntakeReverseAuto;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class ScoreConeHigh extends SequentialCommandGroup{
    public ScoreConeHigh(Elbow elbow, Intake intake, Shoulder shoulder) {
        addCommands(
            new ArmAutoSetpointWait(elbow, shoulder, Constants.Arm.SCOREHIGH.SHOULDER, Constants.Arm.SCOREHIGH.ELBOW).withTimeout(1.5),
            new IntakeReverseAuto(intake).withTimeout(0.3),
            new ArmAutoSetpointNoWait(elbow, shoulder, Constants.Arm.CARRY.SHOULDER, Constants.Arm.CARRY.ELBOW)
        );
    }
}
