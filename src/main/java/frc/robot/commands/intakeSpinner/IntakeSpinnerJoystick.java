package frc.robot.commands.intakeSpinner;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.IntakeSpinnerLamprey;

public class IntakeSpinnerJoystick extends CommandBase {
    private IntakeSpinnerLamprey intakeSpinner;
    private XboxController operatorController;
    public IntakeSpinnerJoystick(IntakeSpinnerLamprey intakeSpinner, XboxController operatorController) {
       this.intakeSpinner = intakeSpinner; 
       this.operatorController = operatorController;
       addRequirements(intakeSpinner);
    }

    @Override
    public void initialize() {
        intakeSpinner.disable();
    }

    @Override
    public void execute() {
        double output = Utilities.deadband(operatorController.getLeftX(), 0.1);
        intakeSpinner.setIntakeSpinnerMotorSpeed(output);
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeSpinner.setIntakeSpinnerMotorSpeed(0);
        intakeSpinner.enable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
