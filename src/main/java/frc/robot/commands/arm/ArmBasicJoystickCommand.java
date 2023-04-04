package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmBasicJoystickCommand extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    Supplier<XboxController> joystick;
    boolean firstTimeThru = true;
    double maxSpeed= 0.2;
    double currentX;


    public ArmBasicJoystickCommand(Elbow elbow, Shoulder shoulder, Supplier<XboxController> joystick) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.joystick = joystick;
        addRequirements(shoulder);

    }

    @Override
    public void initialize() {
        firstTimeThru = true;
        double mathShoulderAngle = shoulder.getShoulderLampreyDegrees();
        double mathElbowAngle = elbow.getElbowLampreyDegrees();
        currentX = Constants.Arm.SHOULDER_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle)) + Constants.Arm.ELBOW_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle) + Units.degreesToRadians(mathElbowAngle));
    }

    @Override
    public void execute() {
        double outputElbow, outputShoulder;
        if (currentX < 0) {
            outputShoulder = Utilities.deadbandAndSquare(-joystick.get().getLeftY(), 0.15);
            outputElbow = Utilities.deadbandAndSquare(joystick.get().getRightY(), 0.15);
        }
        else {
            outputShoulder = Utilities.deadbandAndSquare(-joystick.get().getLeftY(), 0.15);
            outputElbow = Utilities.deadbandAndSquare(-joystick.get().getRightY(), 0.15);
        }
        outputShoulder = MathUtil.clamp(outputShoulder, -maxSpeed, maxSpeed);
        outputElbow = MathUtil.clamp(outputElbow, -maxSpeed, maxSpeed);

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
