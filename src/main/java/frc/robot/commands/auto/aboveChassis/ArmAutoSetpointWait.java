package frc.robot.commands.auto.aboveChassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmAutoSetpointWait extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    IntakeSpinnerLamprey intakespinner;
    ArmPosition armPosition;
    double elbowSetpoint = 0;
    double shoulderSetpoint = 0;
    
    public ArmAutoSetpointWait(Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, ArmPosition armPosition) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.intakespinner = intakespinner;
        this.armPosition = armPosition;
        addRequirements(elbow, shoulder);
    }

    @Override
    public void initialize() {
        shoulder.enable();
        elbow.enable();
        shoulder.setSetpoint(armPosition.shoulder);
        elbow.setSetpoint(armPosition.elbow);
        intakespinner.setPosition(armPosition);
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.enable();
        elbow.enable();
    }

    @Override
    public boolean isFinished() {
       return elbow.atSetpoint() && shoulder.atSetpoint();
    } 
}