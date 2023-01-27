package frc.robot.commands.arm;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class ArmSetpointCommand extends CommandBase {

    Arm arm;
    Supplier<CommandXboxController> joystick;

    double shoulderP = 0.005;
    double shoulderI = 0;
    double shoulderD = 0;
    //Velocity is 68, Acceleration is 
    double elbowP = 0.2;
    double elbowI = 0;
    double elbowD = 0;

    double elbowSetpoint;
    double shoulderSetpoint;
    
    
    //Seems to start slowing down at 45 degrees, will probably have to change due to gearings and such.
    ProfiledPIDController shoulderController = new ProfiledPIDController(shoulderP, shoulderI, shoulderD, new TrapezoidProfile.Constraints(106.3, 0.0001));


    public ArmSetpointCommand(Arm arm, double shoulderSetpoint, double elbowSetpoint) {
        this.arm = arm;
        this.elbowSetpoint = elbowSetpoint;
        this.shoulderSetpoint = shoulderSetpoint;
        addRequirements(arm);

    }


    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        arm.holdShoulderPosition(shoulderSetpoint);
        arm.holdElbowPosition(elbowSetpoint);
        SmartDashboard.putString("hello", "I'll be right there!");
        
    }

    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }



    @Override
    public boolean isFinished() {
        if (arm.shoulderAtSetpoint() && arm.elbowAtSetpoint()) {
            SmartDashboard.putString("hello", "I made it!");
            return true;
        }
        else {
            return false;
        }
        

    }
    
}
