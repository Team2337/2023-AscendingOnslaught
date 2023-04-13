package frc.robot.commands.arm.wakaWaka;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wakaWaka.wakaWakaArmTalon;

public class wakaWakaArmDrive extends CommandBase{
    wakaWakaArmTalon arm;
    double speed;

    public wakaWakaArmDrive(wakaWakaArmTalon arm, double speed) {
        this.arm = arm;
        this.speed = speed;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
        arm.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
