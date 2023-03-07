package frc.robot.commands.arm.intake;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.LEDState;
import frc.robot.subsystems.arm.Intake;

public class IntakeCommand extends CommandBase{
    Intake intake;
    private double speed;
    private boolean firstTimeThru;
    Supplier<GamePiece> gamePiece;
    Consumer<LEDState> ledState;

    public IntakeCommand(Supplier<GamePiece> gamePiece, Consumer<LEDState> ledState, Intake intake) {
        this.gamePiece = gamePiece;
        this.ledState = ledState;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        firstTimeThru = true;
        intake.disable();
    }

    @Override
    public void execute() {
        if (gamePiece.get() == GamePiece.Cone) {
            speed = 1.0;
            if (intake.hasCone() == true) {
                ledState.accept(LEDState.HasGamePiece);
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
        ledState.accept(LEDState.HasGamePiece);
    }
}