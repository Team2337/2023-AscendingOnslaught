package frc.robot.commands.claw;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase{
    private final Claw claw;
    
    public CloseClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }
    
    @Override
    public void initialize() {
        claw.close();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean isFinished) {
        claw.stop();
    }


}