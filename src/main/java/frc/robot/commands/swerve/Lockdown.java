package frc.robot.commands.swerve;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.drive.CartesianProfiledPointToPointLockdown;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Lockdown extends SequentialCommandGroup{
    public Lockdown(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new CartesianProfiledPointToPointLockdown(
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                2.0, 
                3.0, 
                Units.inchesToMeters(10), 
                Units.inchesToMeters(10), 
                autoDrive, 
                drivetrain,
                heading
                )
        );
    }        
}
