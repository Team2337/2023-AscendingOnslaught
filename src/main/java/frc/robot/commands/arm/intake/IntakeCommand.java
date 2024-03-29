package frc.robot.commands.arm.intake;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.LEDState;
import frc.robot.subsystems.arm.Intake;

public class IntakeCommand extends CommandBase{
    Intake intake;
    private double speed;
    private boolean firstTimeThru;
    Supplier<GamePiece> gamePiece;
    Consumer<LEDState> ledState;
    Supplier<Boolean> intakeOverride;

    public IntakeCommand(Supplier<GamePiece> gamePiece, Supplier<Boolean> intakeOverride, Consumer<LEDState> ledState, Intake intake) {
        this.gamePiece = gamePiece;
        this.ledState = ledState;
        this.intake = intake;
        this.intakeOverride = intakeOverride;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        firstTimeThru = true;
    }

    @Override
    public void execute() {
        if (gamePiece.get() == GamePiece.Cone) {
            speed = 1.0;
            if (intake.hasCone() == true && !intakeOverride.get()) {
                ledState.accept(LEDState.HasGamePiece);
                speed = 0.0;
                //TODO: Fix this eventually, make the setpoint the intermediary.
                //shoulder.setSetpoint(Constants.Arm.ArmPosition.CARRY.shoulderCube);
                //elbow.setSetpoint(Constants.Arm.ArmPosition.CARRY.elbowCube);
                //intakespinner.setSetpoint(Constants.Arm.ArmPosition.CARRY.wristCube);   
            }
        } else {
            speed = -1.0;
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