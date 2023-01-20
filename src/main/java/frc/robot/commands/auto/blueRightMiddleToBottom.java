package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class blueRightMiddleToBottom extends SequentialCommandGroup{
    public blueRightMiddleToBottom(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(5),
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueBottomStagingMark, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(15), 
                Units.inchesToMeters(15), 
                autoDrive, 
                heading
                ),
            new WaitCommand(5),
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueGridRightRobotCenter, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(15), 
                Units.inchesToMeters(15), 
                autoDrive, 
                heading
                )
        );
    }        
}
