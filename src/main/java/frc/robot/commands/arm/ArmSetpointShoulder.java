package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmSetpointShoulder extends CommandBase {

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
    boolean isTimedOut = false;
    private static double timeOut = 2;
    double timerStart;
    double timerCurrent;
    
    
    //Seems to start slowing down at 45 degrees, will probably have to change due to gearings and such.
    ProfiledPIDController shoulderController = new ProfiledPIDController(shoulderP, shoulderI, shoulderD, new TrapezoidProfile.Constraints(106.3, 0.0001));

    public ArmSetpointShoulder(ArmPosition armPosition, Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer) {
        this(armPosition, elbow, shoulder, intakespinner, robotContainer, timeOut);
    }

    public ArmSetpointShoulder(ArmPosition armPosition, Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, double timeOut) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.robotContainer = robotContainer;
        this.intakespinner = intakespinner;
        this.armPosition = armPosition;
        this.timeOut = timeOut;
        addRequirements(elbow, shoulder);

    }


    @Override
    public void initialize() {
        timerStart = Timer.getFPGATimestamp() / 1000000;
        if (robotContainer.getGamepiece() == GamePiece.Cone) {
            elbowSetpoint = armPosition.elbowCone;
            shoulder.enable();
            elbow.disable();
            intakespinner.enable();
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
            elbow.disable();
            intakespinner.enable();
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
        intakespinner.setSetpoint(wristSetpoint);
        SmartDashboard.putNumber("Start Timer", timerStart);
    }
    
    
    @Override
    public void execute() {
        timerCurrent = Timer.getFPGATimestamp() / 1000000;
        SmartDashboard.putNumber("Current Timer", Math.round(timerCurrent));
        System.out.println(timerCurrent - timerStart);
        if ((timerCurrent - timerStart) > timeOut) {
            isTimedOut = true;    
        }
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.enable();
        elbow.enable();
        
    }



    @Override
    public boolean isFinished() {
       return shoulder.atSetpoint() || isTimedOut;
        

    }
    
}
