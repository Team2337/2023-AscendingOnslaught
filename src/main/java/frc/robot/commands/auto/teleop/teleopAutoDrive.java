package frc.robot.commands.auto.teleop;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.CartesianProfiledPointToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class teleopAutoDrive extends SequentialCommandGroup{
    public teleopAutoDrive(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading, String allianceColor, int position) {
        addCommands(
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueRightIntermediaryFar, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                1.5, 
                1.5, 
                Units.inchesToMeters(90), 
                Units.inchesToMeters(90), 
                autoDrive, 
                heading
                ),
            new WaitCommand(2),
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueGridRightRobotRight, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                1.5, 
                1.5, 
                Units.inchesToMeters(90), 
                Units.inchesToMeters(90), 
                autoDrive, 
                heading
                )
        );
    }        
}
