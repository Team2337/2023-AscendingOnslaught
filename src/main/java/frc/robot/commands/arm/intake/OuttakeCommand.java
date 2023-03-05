package frc.robot.commands.arm.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.LEDState;
import frc.robot.subsystems.arm.Intake;

public class OuttakeCommand extends CommandBase{
    Intake intake;
    RobotContainer robotContainer;
    private double speed;

    public OuttakeCommand(Intake intake, RobotContainer robotContainer) {
        this.intake = intake;
        this.robotContainer = robotContainer;

        addRequirements(intake);
    }

    @Override
    public void initialize() {       
        
    }

    @Override
    public void execute() {
        if (robotContainer.getGamepiece() == GamePiece.Cone) {
            speed = -0.66;
        } else {
            speed = 0.66;
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
        robotContainer.setLEDState(LEDState.Nothing);
    }
}