package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Test extends SequentialCommandGroup{
    public Test(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(1),
            new CartesianVectorProfileToPointCommand(
                new Translation2d(2, 0), 
                drivetrain::getTranslation,
                1.5,
                Units.inchesToMeters(80),
                autoDrive, 
                heading
            )
        );
    }        
}
