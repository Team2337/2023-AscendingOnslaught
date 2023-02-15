package frc.robot.commands.arm.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Intake;

public class IntakeCommand extends CommandBase{
    Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        //TODO: Confirm intake direction
        
    }

    @Override
    public void execute() {
        if (intake.hasCone() == false) {
            intake.setIntakeSpeed(0.66);
        }
        else {
            intake.setIntakeSpeed(0);
        }
        
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