package frc.robot.commands.arm;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmSetpointWithEnding extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    IntakeSpinnerLamprey intakespinner;
    Supplier<CommandXboxController> joystick;
    RobotContainer robotContainer;
    double tolerance;

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


    public ArmSetpointWithEnding(ArmPosition armPosition, double tolerance, Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer) {
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
        if (robotContainer.getGamepiece() == GamePiece.Cone) {
            elbowSetpoint = armPosition.elbowCone;
            shoulder.enable();
            elbow.enable();
            intakespinner.enable();
            if (armPosition.elbowCone > 155) {
                elbowSetpoint = 155;
            }
            if (armPosition.elbowCone < -155) {
                elbowSetpoint = -155;
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
            if (armPosition.elbowCube > 155) {
                elbowSetpoint = 155;
            }
            if (armPosition.elbowCube < -155) {
                elbowSetpoint = -155;
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
        
        
        shoulder.setSetpoint(shoulderSetpoint);
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
        shoulder.pastPosition = armPosition.toString();
    }



    @Override
    public boolean isFinished() {
       return (Utilities.withinTolerance(shoulder.getSetpoint(), shoulder.getShoulderLampreyDegrees(), tolerance)); //&& Utilities.withinTolerance(elbow.getSetpoint(), elbow.getElbowLampreyDegrees(), 5.0));

    }
    
}

