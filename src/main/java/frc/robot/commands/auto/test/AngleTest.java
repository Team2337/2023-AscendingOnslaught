package frc.robot.commands.auto.test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.CartesianProfiledPointToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class AngleTest extends SequentialCommandGroup{
    public AngleTest(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(1),
            new CartesianProfiledPointToPointCommand(
                new Translation2d(3, 3), 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(30), 
                Units.inchesToMeters(30), 
                autoDrive, 
                heading
                )
        );
    }        
}
