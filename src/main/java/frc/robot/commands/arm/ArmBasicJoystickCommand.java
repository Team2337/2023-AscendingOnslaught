package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmBasicJoystickCommand extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    Supplier<XboxController> joystick;
    boolean firstTimeThru = true;


    public ArmBasicJoystickCommand(Elbow elbow, Shoulder shoulder, Supplier<XboxController> joystick) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.joystick = joystick;
        addRequirements(shoulder);

    }

    @Override
    public void initialize() {
        firstTimeThru = true;
        shoulder.disable();
        elbow.disable();

        
    }

    @Override
    public void execute() {
        double outputElbow, outputShoulder;
        if (shoulder.getShoulderLampreyDegrees() > 90) {
            outputShoulder = Utilities.deadbandAndSquare(-joystick.get().getLeftY(), 0.15);
            outputElbow = Utilities.deadbandAndSquare(joystick.get().getRightY(), 0.15);
        }
        else {
            outputShoulder = Utilities.deadbandAndSquare(-joystick.get().getLeftY(), 0.15);
            outputElbow = Utilities.deadbandAndSquare(-joystick.get().getRightY(), 0.15);
        }
        
        if ((outputShoulder == 0) && (outputElbow == 0)){ 
            if (firstTimeThru) {
                SmartDashboard.putString("Are we holding?", "yes!");
                //elbow.holdElbowPosition(elbow.getElbowPositionTicks());
                //shoulder.holdShoulderPosition(shoulder.getShoulderPositionTicks());
                elbow.enable();
                shoulder.enable();
                if (Utilities.withinTolerance(elbow.getSetpoint(), elbow.getElbowLampreyDegrees(), 4)) {
                    elbow.setSetpoint(elbow.getSetpoint());
                } else {
                    elbow.setSetpoint(elbow.getElbowLampreyDegrees());
                }
                
                shoulder.setSetpoint(shoulder.getShoulderLampreyDegrees());

                firstTimeThru = false;
            }
        } else {
            elbow.disable();
            shoulder.disable();
            SmartDashboard.putString("Are we holding?", "no!");
            elbow.setElbowSpeed(outputElbow);
            shoulder.setShoulderSpeed(outputShoulder);
            firstTimeThru = true;
        }
    }
        

    @Override
    public void end(boolean interrupted) {
        shoulder.enable();
        elbow.enable();
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
