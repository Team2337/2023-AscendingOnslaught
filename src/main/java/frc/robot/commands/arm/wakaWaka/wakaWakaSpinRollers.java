package frc.robot.commands.arm.wakaWaka;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wakaWaka.wakaWakaArm;
import frc.robot.subsystems.wakaWaka.wakaWakaIntake;

public class wakaWakaSpinRollers extends CommandBase {
    wakaWakaIntake intake;
    double speed;

    public wakaWakaSpinRollers(wakaWakaIntake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        intake.setIntakeSpeed(speed);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}