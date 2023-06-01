package frc.robot.commands.arm.wakaWaka;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wakaWaka.wakaWakaArm;

public class wakaWakaArmDrive extends CommandBase{
    wakaWakaArm arm;
    double speed;

    public wakaWakaArmDrive(wakaWakaArm arm, double speed) {
        this.arm = arm;
        this.speed = speed;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.disable();
    }
    @Override
    public void execute() {
        arm.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        arm.changeEncoderPosition(0);
    }

    @Override
    public boolean isFinished() {
        return arm.getArmLimit();
    }
}
