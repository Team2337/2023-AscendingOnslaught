package frc.robot.commands.auto.aboveChassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmAutoSetpointNoWait extends InstantCommand {

    Elbow elbow;
    Shoulder shoulder;
    double elbowSetpoint = 0;
    double shoulderSetpoint = 0;
    
    public ArmAutoSetpointNoWait(Elbow elbow, Shoulder shoulder, double shoulderSetpoint, double elbowSetpoint) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.elbowSetpoint = elbowSetpoint;
        this.shoulderSetpoint = shoulderSetpoint;
        addRequirements(elbow, shoulder);
    }

    @Override
    public void initialize() {
        shoulder.enable();
        elbow.enable();
        shoulder.setSetpoint(shoulderSetpoint);
        elbow.setSetpoint(elbowSetpoint);
    }
    
}
