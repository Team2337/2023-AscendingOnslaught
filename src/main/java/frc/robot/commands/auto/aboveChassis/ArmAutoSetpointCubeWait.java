package frc.robot.commands.auto.aboveChassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmAutoSetpointCubeWait extends CommandBase {

    Elbow elbow;
    Shoulder shoulder;
    IntakeSpinnerLamprey intakespinner;
    ArmPosition armPosition;
    double elbowSetpoint = 0;
    double shoulderSetpoint = 0;
    double speed = 0.7;
    
    public ArmAutoSetpointCubeWait(Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, ArmPosition armPosition) {
        this(0.7, elbow, shoulder, intakespinner, armPosition);
    }

    public ArmAutoSetpointCubeWait(double speed, Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, ArmPosition armPosition) {
        this.speed = speed;
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.intakespinner = intakespinner;
        this.armPosition = armPosition;
        addRequirements(elbow, shoulder);
    }

    @Override
    public void initialize() {
        shoulder.enable();
        elbow.enable();
        shoulder.setShoulderSpeed(speed);
        elbow.setElbowSpeed(speed);
        shoulder.setSetpoint(armPosition.shoulderCube);
        elbow.setSetpoint(armPosition.elbowCube);
        intakespinner.setSetpoint(armPosition.wristCube);
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.enable();
        elbow.enable();
    }

    @Override
    public boolean isFinished() {
       return elbow.atSetpoint() && shoulder.atSetpoint();
    } 
}