package frc.robot.commands.intakeSpinner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSpinnerLamprey;

public class IntakeSpinnerSetPoint extends InstantCommand {
    private IntakeSpinnerLamprey intakeSpinner;
    private double setpoint; 

    public IntakeSpinnerSetPoint(IntakeSpinnerLamprey intakeSpinner, double setpoint) {
        this.intakeSpinner = intakeSpinner;
        this.setpoint = setpoint;
        addRequirements(intakeSpinner);
    }

    @Override
    public void initialize() {
        intakeSpinner.enable();
        intakeSpinner.setSetpoint(setpoint);
    }

}
