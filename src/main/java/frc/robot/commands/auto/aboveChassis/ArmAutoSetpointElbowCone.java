package frc.robot.commands.auto.aboveChassis;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmAutoSetpointElbowCone extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    IntakeSpinnerLamprey intakespinner;
    Supplier<CommandXboxController> joystick;
    RobotContainer robotContainer;

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
    double wristSetpoint = 0;
    ArmPosition armPosition;
    double tolerance;
    
    
   

    public ArmAutoSetpointElbowCone(ArmPosition armPosition, double tolerance, Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.robotContainer = robotContainer;
        this.intakespinner = intakespinner;
        this.armPosition = armPosition;
        this.tolerance = tolerance;
        addRequirements(elbow, shoulder);

    }


    @Override
    public void initialize() {
        elbowSetpoint = armPosition.elbowCone;
        shoulder.disable();
        elbow.enable();
        intakespinner.enable();
        if (armPosition.elbowCone > Constants.Arm.ELBOW_LIMIT) {
            elbowSetpoint = Constants.Arm.ELBOW_LIMIT;
        }
        if (armPosition.elbowCone < -Constants.Arm.ELBOW_LIMIT) {
            elbowSetpoint = -Constants.Arm.ELBOW_LIMIT;
        }
        if (armPosition.wristCone < Constants.Arm.WRIST_LOWER_LIMIT) {
            wristSetpoint = Constants.Arm.WRIST_LOWER_LIMIT;
        }
        if (armPosition.wristCone > Constants.Arm.WRIST_UPPER_LIMIT) {
            wristSetpoint = Constants.Arm.WRIST_UPPER_LIMIT;
        }
        shoulderSetpoint = armPosition.shoulderCone;
        wristSetpoint = armPosition.wristCone;
        
        elbow.setSetpoint(elbowSetpoint);
        intakespinner.setSetpoint(wristSetpoint);
    }
    
    
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.enable();
        elbow.enable();
        shoulder.pastPosition = armPosition;
    }



    @Override
    public boolean isFinished() {
       return Utilities.withinTolerance(elbow.getSetpoint(), elbow.getElbowLampreyDegrees(), tolerance);
        

    }
    
}
