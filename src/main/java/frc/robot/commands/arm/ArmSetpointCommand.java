package frc.robot.commands.arm;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmSetpointCommand extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
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


    public ArmSetpointCommand(Elbow elbow, Shoulder shoulder, double shoulderSetpoint, double elbowSetpoint) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.elbowSetpoint = elbowSetpoint;
        this.shoulderSetpoint = shoulderSetpoint;
        addRequirements(elbow, shoulder);

    }


    @Override
    public void initialize() {
        shoulder.enable();
        elbow.enable();
        if (elbowSetpoint > 160) {
            elbowSetpoint = 160;
        }
        if (elbowSetpoint < -160) {
            elbowSetpoint = -160;
        }
        
        shoulder.setSetpoint(shoulderSetpoint);
        elbow.setSetpoint(elbowSetpoint);
    }

    
    @Override
    public void execute() {
    
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.enable();
        elbow.enable();
        
    }



    @Override
    public boolean isFinished() {
       return (false);
        

    }
    
}
