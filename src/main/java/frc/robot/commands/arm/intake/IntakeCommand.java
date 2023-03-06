package frc.robot.commands.arm.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class IntakeCommand extends CommandBase{
    Intake intake;
    RobotContainer robotContainer;
    Shoulder shoulder;
    Elbow elbow;
    IntakeSpinnerLamprey intakespinner;
    private double speed;
    private boolean firstTimeThru;

    public IntakeCommand(Intake intake, RobotContainer robotContainer, Shoulder shoulder, Elbow elbow, IntakeSpinnerLamprey intakespinner) {
        this.intake = intake;
        this.robotContainer = robotContainer;
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.intakespinner = intakespinner;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        firstTimeThru = true;
    }

    @Override
    public void execute() {
        if (robotContainer.getGamepiece() == GamePiece.Cone) {
            speed = 0.66;
            if (intake.hasCone() == true) {
                speed = 0.0;
                //TODO: Fix this eventually, make the setpoint the intermediary.
                //shoulder.setSetpoint(Constants.Arm.ArmPosition.CARRY.shoulderCube);
                //elbow.setSetpoint(Constants.Arm.ArmPosition.CARRY.elbowCube);
                //intakespinner.setSetpoint(Constants.Arm.ArmPosition.CARRY.wristCube);   
            }
        } else {
            speed = -0.66;
        }
        
        intake.setIntakeSpeed(speed);
    }
        

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
    }
}