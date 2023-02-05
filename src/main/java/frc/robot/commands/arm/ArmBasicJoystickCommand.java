package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmBasicJoystickCommand extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    Supplier<XboxController> joystick;
    boolean shouldHoldArm = true;


    public ArmBasicJoystickCommand(Elbow elbow, Shoulder shoulder, Supplier<XboxController> joystick) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.joystick = joystick;
        addRequirements(shoulder);

    }

    @Override
    public void initialize() {
        shouldHoldArm = true;
        shoulder.disable();
        elbow.disable();

        
    }

    @Override
    public void execute() {
        double outputShoulder = Utilities.deadband(-joystick.get().getLeftY(), 0.15);
        double outputElbow = Utilities.deadband(-joystick.get().getRightY(), 0.15);
        if ((outputShoulder == 0) && (outputElbow == 0)){ 
            if (shouldHoldArm) {
                elbow.holdElbowPosition(elbow.getElbowPositionTicks());
                shoulder.holdShoulderPosition(shoulder.getShoulderPositionTicks());
                shouldHoldArm = false;
            }
        } else {
            elbow.setElbowSpeed(outputElbow);
            shoulder.setShoulderSpeed(outputShoulder);
            shouldHoldArm = true;
        }
    }
       // m_arm.setShoulderSpeed(deadband(-joystick.get().getLeftY(), 0.2));
       // m_arm.setElbowSpeed(deadband(-joystick.get().getRightY(), 0.2));
        

    @Override
    public void end(boolean interrupted) {
        shoulder.enable();
        elbow.enable();
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
