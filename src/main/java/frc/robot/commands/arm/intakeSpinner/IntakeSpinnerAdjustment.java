package frc.robot.commands.arm.intakeSpinner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSpinnerLamprey;

public class IntakeSpinnerAdjustment extends InstantCommand {
    private IntakeSpinnerLamprey intakeSpinner;
    private double setpoint;
    private double angleChange;
    public IntakeSpinnerAdjustment(IntakeSpinnerLamprey intakeSpinner, double angleChange) {
       this.intakeSpinner = intakeSpinner; 
       this.angleChange = angleChange;
       addRequirements(intakeSpinner);
    }

    @Override
    public void initialize() {
        intakeSpinner.enable();
        setpoint = intakeSpinner.getSetpoint();
        setpoint = setpoint + angleChange;
        if (setpoint< Constants.Arm.WRIST_LOWER_LIMIT) {
            setpoint = Constants.Arm.WRIST_LOWER_LIMIT;
        }
        if (setpoint > Constants.Arm.WRIST_UPPER_LIMIT) {
            setpoint = Constants.Arm.WRIST_UPPER_LIMIT;
        }
        intakeSpinner.setSetpoint(setpoint);
    }

}

