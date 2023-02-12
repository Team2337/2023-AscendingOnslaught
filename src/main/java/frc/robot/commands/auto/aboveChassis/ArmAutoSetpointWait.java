package frc.robot.commands.auto.aboveChassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmAutoSetpointWait extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    double elbowSetpoint = 0;
    double shoulderSetpoint = 0;
    
    public ArmAutoSetpointWait(Elbow elbow, Shoulder shoulder, double shoulderSetpoint, double elbowSetpoint) {
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