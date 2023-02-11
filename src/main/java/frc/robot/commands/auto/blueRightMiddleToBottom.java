package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.drive.CartesianProfiledPointToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class blueRightMiddleToBottom extends SequentialCommandGroup{
    public blueRightMiddleToBottom(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(2),
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueBottomStagingMark, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                1.5, 
                1.5, 
                Units.inchesToMeters(90), 
                Units.inchesToMeters(90), 
                autoDrive, 
                heading
                ),
            new WaitCommand(1),
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueGridRightRobotCenter, 
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
