package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class blueMiddleMiddleScoreC5Balance extends SequentialCommandGroup{
    public blueMiddleMiddleScoreC5Balance(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(2),
            // new ScoreGameObjectCommand
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueCenterOfChargeStation, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading)
        );
    }        
}
