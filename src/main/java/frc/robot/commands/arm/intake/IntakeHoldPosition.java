package frc.robot.commands.arm.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Intake;


public class IntakeHoldPosition extends CommandBase{
    Intake intake;



    public IntakeHoldPosition(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.holdPosition(intake.getIntakePosition());
    }

    @Override
    public void execute() {
      
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