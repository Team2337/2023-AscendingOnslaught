package frc.robot.commands.auto.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.CartesianProfiledPointToPointWithRotation;
import frc.robot.commands.auto.drive.CartesianVectorProfileToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Test extends SequentialCommandGroup{
    public Test(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new CartesianProfiledPointToPointWithRotation(new Translation2d(2, 0), new Rotation2d().fromDegrees(45), drivetrain::getTranslation, drivetrain::getRotation, 1.0, 1.0, 2, 2, autoDrive, drivetrain, heading)
        );
    }        
}
