package frc.robot.commands.auto.aboveChassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmAutoSetpointCubeNoWait extends InstantCommand {

    Elbow elbow;
    Shoulder shoulder;
    IntakeSpinnerLamprey intakespinner;
    ArmPosition armPosition;
    double elbowSetpoint = 0;
    double shoulderSetpoint = 0;
    double speed = 0.7;

    public ArmAutoSetpointCubeNoWait(Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, ArmPosition armPosition) {
        this(0.7, elbow, shoulder, intakespinner, armPosition);
    }
    
    public ArmAutoSetpointCubeNoWait(double speed, Elbow elbow, Shoulder shoulder, IntakeSpinnerLamprey intakespinner, ArmPosition armPosition) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.intakespinner = intakespinner;
        this.armPosition = armPosition;
        this.speed = speed;
        addRequirements(elbow, shoulder);
    }

    @Override
    public void initialize() {
        shoulder.enable();
        elbow.enable();
        // shoulder.changePeakOutput(speed);
        // elbow.changePeakOutput(speed);
        shoulder.setSetpoint(armPosition.shoulderCube);
        elbow.setSetpoint(armPosition.elbowCube);
        intakespinner.setSetpoint(armPosition.wristCube);
    }
    
}
