package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmBasicJoystickCommand extends CommandBase {

    Arm m_arm;
    Supplier<XboxController> joystick;
    boolean shouldHoldArm = true;


    public ArmBasicJoystickCommand(Arm arm, Supplier<XboxController> joystick) {
        m_arm = arm;
        this.joystick = joystick;
        addRequirements(m_arm);

    }

    @Override
    public void initialize() {
        shouldHoldArm = true;
        
    }

    @Override
    public void execute() {
        double outputShoulder = deadband(-joystick.get().getLeftY(), 0.15);
        double outputElbow = deadband(-joystick.get().getRightY(), 0.15);
        if ((outputShoulder == 0) && (outputElbow == 0)){ 
            if (shouldHoldArm) {
                m_arm.holdElbowPosition(m_arm.getElbowPositionTicks());
                m_arm.holdShoulderPosition(m_arm.getShoulderPositionTicks());
                shouldHoldArm = false;
            }
        } else {
            m_arm.setElbowSpeed(outputElbow);
            m_arm.setShoulderSpeed(outputShoulder);
            shouldHoldArm = true;
        }
    }
       // m_arm.setShoulderSpeed(deadband(-joystick.get().getLeftY(), 0.2));
       // m_arm.setElbowSpeed(deadband(-joystick.get().getRightY(), 0.2));
        

    public double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance) {
            return 0.0;
        }
        else {
           return ((value - tolerance) / (1 - tolerance)) / 10.0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
