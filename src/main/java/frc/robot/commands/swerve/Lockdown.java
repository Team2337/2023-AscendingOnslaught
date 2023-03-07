package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.drive.CartesianProfiledPointToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Lockdown extends SequentialCommandGroup{
    public Lockdown(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new CartesianProfiledPointToPointCommand(
                new Translation2d(drivetrain.getPose().getX(), drivetrain.getPose().getY()), 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                1.5, 
                1.5, 
                Units.inchesToMeters(90), 
                Units.inchesToMeters(90), 
                autoDrive, 
                drivetrain,
                heading
                )
        );
    }        
}
