package frc.robot.commands.arm;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmSetpointCommand extends CommandBase {

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
    
    
    //Seems to start slowing down at 45 degrees, will probably have to change due to gearings and such.
    ProfiledPIDController shoulderController = new ProfiledPIDController(shoulderP, shoulderI, shoulderD, new TrapezoidProfile.Constraints(106.3, 0.0001));


    public ArmSetpointCommand(ArmPosition armPosition, Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.robotContainer = robotContainer;
        this.intakespinner = intakespinner;
        this.armPosition = armPosition;
        addRequirements(elbow, shoulder);

    }


    @Override
    public void initialize() {
        shoulder.pastPosition = armPosition;
        if (robotContainer.getGamepiece() == GamePiece.Cone) {
            elbowSetpoint = armPosition.elbowCone;
            shoulder.enable();
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

        } else {
            elbowSetpoint = armPosition.elbowCube;
            shoulder.enable();
            elbow.enable();
            intakespinner.enable();
            if (armPosition.elbowCube > Constants.Arm.ELBOW_LIMIT) {
                elbowSetpoint = Constants.Arm.ELBOW_LIMIT;
            }
            if (armPosition.elbowCube < -Constants.Arm.ELBOW_LIMIT) {
                elbowSetpoint = -Constants.Arm.ELBOW_LIMIT;
            }
            if (armPosition.wristCube < Constants.Arm.WRIST_LOWER_LIMIT) {
                wristSetpoint = Constants.Arm.WRIST_LOWER_LIMIT;
            }
            if (armPosition.wristCube > Constants.Arm.WRIST_UPPER_LIMIT) {
                wristSetpoint = Constants.Arm.WRIST_UPPER_LIMIT;
            }
            shoulderSetpoint = armPosition.shoulderCube;
            wristSetpoint = armPosition.wristCube;
        }
        
        elbowP = 0.018 + 0.013 * Math.abs(Math.cos(Units.degreesToRadians(elbowSetpoint + shoulderSetpoint)));
        
        shoulder.setSetpoint(shoulderSetpoint);
        elbow.setElbowSetpoint(elbowSetpoint, elbowP);
        intakespinner.setSetpoint(wristSetpoint);
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
