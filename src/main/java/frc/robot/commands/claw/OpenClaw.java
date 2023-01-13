package frc.robot.commands.claw;


import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase{
    private final Claw claw;
    // private double startTime;
    // private double clawTimer = 1;

    public OpenClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }
    
    @Override
    public void initialize() {
        claw.open();
        // this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // if (Timer.getFPGATimestamp() > startTime + clawTimer) {
        //     return true;
        // } else {
            return false;
        // }
    }

    @Override
    public void end(boolean isFinished) {
        claw.stop();
    }
}