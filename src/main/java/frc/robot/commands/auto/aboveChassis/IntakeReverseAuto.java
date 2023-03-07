package frc.robot.commands.auto.aboveChassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Intake;

public class IntakeReverseAuto extends CommandBase{
    Intake intake;

    public IntakeReverseAuto(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeSpeed(Constants.Auto.intakeAutoReverseSpeed);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
    }
}
