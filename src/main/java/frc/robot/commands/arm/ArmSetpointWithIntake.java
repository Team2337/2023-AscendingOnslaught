package frc.robot.commands.arm;


import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmSetpointWithIntake extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    IntakeSpinnerLamprey intakespinner;
    Supplier<CommandXboxController> joystick;
    Supplier<GamePiece> gamePiece;
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

    public ArmSetpointWithIntake(ArmPosition armPosition, Supplier<GamePiece> gamePiece, Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.intakespinner = intakespinner;
        this.armPosition = armPosition;
        this.gamePiece = gamePiece;
        addRequirements(elbow, shoulder);

    }


    @Override
    public void initialize() {
        if (gamePiece.get() == GamePiece.Cone && shoulder.pastPosition == ArmPosition.SUBSTATION) {
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

            shoulder.setSetpoint(shoulderSetpoint);
            elbow.setSetpoint(elbowSetpoint);
            intakespinner.setSetpoint(wristSetpoint);
        } else if (shoulder.pastPosition == ArmPosition.SUBSTATION) {
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

            shoulder.setSetpoint(shoulderSetpoint);
            elbow.setSetpoint(elbowSetpoint);
            intakespinner.setSetpoint(wristSetpoint);
        }
        
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
       return (false);
        

    }
    
}
