package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmDemoCommand extends SequentialCommandGroup {
    Elbow elbow;
    Shoulder shoulder;
    public ArmDemoCommand(Elbow elbow, Shoulder shoulder) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        addCommands(
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.SUBSTATION.SHOULDER, Constants.Arm.SUBSTATION.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.CARRY.SHOULDER, Constants.Arm.CARRY.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.SCOREHIGH.SHOULDER, Constants.Arm.SCOREHIGH.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.CARRY.SHOULDER, Constants.Arm.CARRY.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.SUBSTATION.SHOULDER, Constants.Arm.SUBSTATION.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.SCOREMID.SHOULDER, Constants.Arm.SCOREMID.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.CARRY.SHOULDER, Constants.Arm.CARRY.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.TELESTANDINGCONE.SHOULDER, Constants.Arm.TELESTANDINGCONE.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.CARRY.SHOULDER, Constants.Arm.SUBSTATION.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.SUBSTATION.SHOULDER, Constants.Arm.SUBSTATION.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.CARRY.SHOULDER, Constants.Arm.CARRY.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.SCORELOW.SHOULDER, Constants.Arm.SCORELOW.ELBOW),
            new WaitCommand(0.5),
            new ArmSetpointCommand(elbow, shoulder, Constants.Arm.CARRY.SHOULDER, Constants.Arm.CARRY.ELBOW),
            new WaitCommand(0.5)

            


        );
    }
    
}
