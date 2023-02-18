package frc.robot.commands.arm.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.arm.Intake;

public class IntakeCommand extends CommandBase{
    Intake intake;
    RobotContainer robotContainer;

    public IntakeCommand(Intake intake, RobotContainer robotContainer) {
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
            if (intake.hasCone() == false) {
                intake.setIntakeSpeed(0.66);
            }
            else {
                intake.setIntakeSpeed(0);
            }    
        } else {
            intake.setIntakeSpeed(-0.66);
        }
        
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