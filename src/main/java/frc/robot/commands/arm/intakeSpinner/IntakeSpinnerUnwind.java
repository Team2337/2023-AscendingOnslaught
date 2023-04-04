package frc.robot.commands.arm.intakeSpinner;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSpinnerLamprey;

public class IntakeSpinnerUnwind extends CommandBase {

    IntakeSpinnerLamprey intakeSpinner;
    Supplier<XboxController> joystick;
    Supplier<Boolean> povUp;
    Supplier<Boolean> povDown;

    boolean firstTimeThru = true;
    double maxSpeed= 0.2;


    public IntakeSpinnerUnwind(IntakeSpinnerLamprey intakeSpinner, Supplier<Boolean> povUp, Supplier<Boolean> povDown) {
        this.intakeSpinner = intakeSpinner;
        this.povUp = povUp;
        this.povDown = povDown;
        addRequirements(intakeSpinner);

    }

    @Override
    public void initialize() {
        firstTimeThru = true;
    }

    @Override
    public void execute() {
        double output = 0;
        if (povUp.get()) {
            output = 0.1;
        }
        if (povDown.get()) {
            output = -0.1;
        }

        if (!(povUp.get()) && !(povDown.get())){ 
            if (firstTimeThru) {
                SmartDashboard.putString("Are we holding?", "yes!");
                //elbow.holdElbowPosition(elbow.getElbowPositionTicks());
                //shoulder.holdShoulderPosition(shoulder.getShoulderPositionTicks());
                intakeSpinner.enable();
                intakeSpinner.setSetpoint(intakeSpinner.getEncoderDegrees());
                firstTimeThru = false;
            }
        } else {
            intakeSpinner.disable();
            intakeSpinner.setIntakeSpinnerMotorSpeed(output);
            firstTimeThru = true;
        }
    }
        

    @Override
    public void end(boolean interrupted) {
        intakeSpinner.enable();
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
