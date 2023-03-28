package frc.robot.commands.arm.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Intake;


public class IntakeUnjam extends CommandBase{
    Intake intake;



    public IntakeUnjam(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeSpeed(0);
        intake.holdPosition(intake.getIntakePosition()-5);
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
        
    }
}