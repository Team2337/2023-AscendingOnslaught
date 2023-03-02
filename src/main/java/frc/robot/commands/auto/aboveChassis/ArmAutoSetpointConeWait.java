package frc.robot.commands.auto.aboveChassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmAutoSetpointConeWait extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    IntakeSpinnerLamprey intakespinner;
    RobotContainer robotContainer;
    ArmPosition armPosition;
    double elbowSetpoint = 0;
    double shoulderSetpoint = 0;
    
    public ArmAutoSetpointConeWait(Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, ArmPosition armPosition) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.intakespinner = intakespinner;
        this.robotContainer = robotContainer;
        this.armPosition = armPosition;
        addRequirements(elbow, shoulder);
    }

    @Override
    public void initialize() {
        shoulder.enable();
        elbow.enable();
        shoulder.setSetpoint(armPosition.shoulderCone);
        elbow.setSetpoint(armPosition.elbowCone);
        intakespinner.setSetpoint(armPosition.wristCone);
        robotContainer.setGamePiece(GamePiece.Cone);
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.enable();
        elbow.enable();
        robotContainer.setGamePiece(GamePiece.Cone);
    }

    @Override
    public boolean isFinished() {
       return elbow.atSetpoint() && shoulder.atSetpoint();
    } 
}