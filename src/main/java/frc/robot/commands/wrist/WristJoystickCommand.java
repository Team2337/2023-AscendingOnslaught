package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class WristJoystickCommand extends CommandBase {
    private final Wrist wrist;
    private final XboxController controller;

    public WristJoystickCommand(Wrist wrist, XboxController controller){
        this.wrist = wrist;
        this.controller = controller;
        addRequirements(wrist);
    }

    @Override
    public void execute(){
        double output = controller.getRightX(); 
        wrist.setSpeed(output);
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stop();
    }
}
