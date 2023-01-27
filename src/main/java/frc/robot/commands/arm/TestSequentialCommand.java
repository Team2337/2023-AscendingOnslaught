package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;

public class TestSequentialCommand extends SequentialCommandGroup{
    Arm arm;
    public TestSequentialCommand(Arm arm) {
        this.arm = arm;
        addCommands(
            new ArmSetpointCommand(arm, -40000, 17500),
            new WaitCommand(2),
            new ArmSetpointCommand(arm, -13000, -27000)


        );
    }
    
}
