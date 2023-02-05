package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmDemoCommand extends SequentialCommandGroup {
    Elbow elbow;
    Shoulder shoulder;
    public ArmDemoCommand(Elbow elbow, Shoulder shoulder) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        addCommands(
            new ArmSetpointCommand(elbow, shoulder, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -13000, -27000), // Substation Shelf
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, 5500, 28000), //Score High
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -40000, -40000), //Ground Pickup
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -2000, 46500), //Score Mid
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -13000, -27000), //Substation Shelf
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, 10000, 72500), //Score low
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, -40000, 17500), //Carry
            new WaitCommand(.5),
            new ArmSetpointCommand(elbow, shoulder, 0, 0)
            


        );
    }
    
}
