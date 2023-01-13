package frc.robot.commands.intake;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class StopIntake extends InstantCommand {
  private final Intake intake;
    public StopIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        intake.stop();
    }
   

}