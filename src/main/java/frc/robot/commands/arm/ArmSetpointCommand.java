package frc.robot.commands.arm;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
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

    double ticksPerDegree = 426.6667;



    double elbowSetpoint = 0;
    double shoulderSetpoint = 0;
    
    
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
        arm.holdShoulderPosition(shoulderSetpoint + Constants.SHOULDER_OFFSET_FOR_PREMADE_SETPOINTS_IN_TICKS);
        arm.holdElbowPosition(elbowSetpoint + Constants.ELBOW_OFFSET_FOR_PREMADE_SETPOINTS_IN_TICKS, 0);
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
