package frc.robot.commands.arm.wakaWaka;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wakaWaka.wakaWakaIntake;

public class wakaWakaHoldRollers extends CommandBase {
    wakaWakaIntake intake;
    double speed;

    public wakaWakaHoldRollers(wakaWakaIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        intake.holdPosition();
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