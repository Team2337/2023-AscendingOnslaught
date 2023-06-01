package frc.robot.commands.arm.wakaWaka;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.CTREUtils;
import frc.robot.subsystems.wakaWaka.wakaWakaArm;

public class wakaWakaDefaultDrive extends CommandBase{
    wakaWakaArm arm;
    double speed;

    public wakaWakaDefaultDrive(wakaWakaArm arm, double speed) {
        this.arm = arm;
        this.speed = speed;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.disable();
        arm.setCurrentLimit(CTREUtils.wakaUpCurrentLimit());
        
    }
    @Override
    public void execute() {
        if (arm.getArmLimit() == false) {
            arm.setSpeed(speed);
        }
        else {
            arm.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.changeEncoderPosition(0);
        arm.setCurrentLimit(CTREUtils.wakaCurrentLimit());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
