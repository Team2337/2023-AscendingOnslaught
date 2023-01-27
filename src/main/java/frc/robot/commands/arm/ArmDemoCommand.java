package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;

public class ArmDemoCommand extends SequentialCommandGroup{
    Arm arm;
    public ArmDemoCommand(Arm arm) {
        this.arm = arm;
        addCommands(
            new ArmSetpointCommand(arm, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -13000, -27000), // Substation Shelf
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, 5500, 28000), //Score High
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -40000, -40000), //Ground Pickup
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -2000, 46500), //Score Mid
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -13000, -27000), //Substation Shelf
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, 10000, 72500), //Score low
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(arm, 0, 0)
            


        );
    }
    
}
