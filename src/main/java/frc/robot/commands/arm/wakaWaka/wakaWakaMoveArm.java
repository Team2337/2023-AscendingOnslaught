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
        arm.enable();
        arm.setSetpoint(setpoint);
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
