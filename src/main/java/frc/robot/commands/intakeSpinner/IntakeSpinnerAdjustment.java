package frc.robot.commands.intakeSpinner;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.IntakeSpinnerLamprey;

public class IntakeSpinnerAdjustment extends InstantCommand {
    private IntakeSpinnerLamprey intakeSpinner;
    private XboxController operatorController;
    private double setpoint;
    private double angleChange;
    public IntakeSpinnerAdjustment(IntakeSpinnerLamprey intakeSpinner, double angleChange) {
       this.intakeSpinner = intakeSpinner; 
       this.angleChange = angleChange;
       addRequirements(intakeSpinner);
    }

    @Override
    public void initialize() {
        setpoint = intakeSpinner.getSetpoint();
        setpoint = setpoint + angleChange;
        intakeSpinner.setSetpoint(setpoint);
    }

}

