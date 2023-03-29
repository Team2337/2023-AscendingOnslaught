package frc.robot.commands.auto.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceIndicatorFront extends CommandBase {
    
    private Supplier<Rotation2d> pitchSupplier;
    private boolean isElevated = false; 


    public AutoBalanceIndicatorFront(Supplier<Rotation2d> pitchSupplier) {
        this.pitchSupplier = pitchSupplier;
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
  
      if (pitchSupplier.get().getDegrees() < -12) {
        isElevated = true;
      }
    }

    @Override
    public void end(boolean interrupted) {
  
    }
  
    @Override
    public boolean isFinished() {
      return (isElevated && pitchSupplier.get().getDegrees() > -9.8);
    }
    
}