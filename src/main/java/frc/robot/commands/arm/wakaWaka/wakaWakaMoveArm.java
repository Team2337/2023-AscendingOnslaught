package frc.robot.commands.arm.wakaWaka;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wakaWaka.wakaWakaArm;

public class wakaWakaMoveArm extends CommandBase {
    wakaWakaArm arm;
    double setpoint;

    public wakaWakaMoveArm(wakaWakaArm arm, double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        arm.setSetpoint(setpoint);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
