package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.intake.IntakeUnjam;
import frc.robot.commands.arm.intakeSpinner.IntakeSpinnerSetPoint;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Intake;

public class UnjamWrist extends SequentialCommandGroup {
    public UnjamWrist(Intake intake, IntakeSpinnerLamprey intakeSpinner, RobotContainer robotContainer) {
        addCommands(
            new IntakeSpinnerSetPoint(intakeSpinner, intakeSpinner.getEncoderDegrees()+ 10.0).andThen(new WaitCommand(0.5)),
            new IntakeUnjam(intake).withTimeout(0.25)
            // new ArmSetpointWithEnding(shoulder.pastPosition, 5, elbow, shoulder, intakeSpinner, robotContainer)
        );
    }
    
}
