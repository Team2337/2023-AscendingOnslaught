package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class blueStartRightyRightScoreC1GBotScoreC2GMidScoreC3 extends SequentialCommandGroup{
    public blueStartRightyRightScoreC1GBotScoreC2GMidScoreC3(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(1),
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueBottomStagingMark, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(30), 
                Units.inchesToMeters(30), 
                autoDrive, 
                heading
                ),
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueGridRightRobotCenter, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(30), 
                Units.inchesToMeters(30), 
                autoDrive, 
                heading
                ),
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueMiddleStagingMark, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(30), 
                Units.inchesToMeters(30), 
                autoDrive, 
                heading
                ),
            new CartesianProfiledPointToPointCommand(
                Constants.Auto.blueGridRightRobotLeft, 
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
