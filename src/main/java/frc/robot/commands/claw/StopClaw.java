package frc.robot.commands.claw;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class StopClaw extends InstantCommand{
    private final Claw claw;
    
    public StopClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }
    
    @Override
    public void initialize() {
        claw.stop();
    }

}