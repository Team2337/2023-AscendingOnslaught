package frc.robot.commands.auto.aboveChassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmAutoShoulderConeNoWait extends InstantCommand {

    Elbow elbow;
    Shoulder shoulder;
    IntakeSpinnerLamprey intakespinner;
    ArmPosition armPosition;
    double elbowSetpoint = 0;
    double shoulderSetpoint = 0;
    
    public ArmAutoShoulderConeNoWait(Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, ArmPosition armPosition) {
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
        shoulder.setSetpoint(armPosition.shoulderCone);
        elbow.setSetpoint(armPosition.elbowCone);
        intakespinner.setSetpoint(armPosition.wristCone);
    }
    
}
