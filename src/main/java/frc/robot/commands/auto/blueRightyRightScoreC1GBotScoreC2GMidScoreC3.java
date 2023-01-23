package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class blueRightyRightScoreC1GBotScoreC2GMidScoreC3 extends SequentialCommandGroup{
    public blueRightyRightScoreC1GBotScoreC2GMidScoreC3(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(2),
            // new ScoreGameObjectCommand
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueBottomStagingMark, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            // new IntakeCommand
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueGridRightRobotCenter, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            // new ScoreGameObjectCommand
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueRightIntermediaryFar, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueMiddleStagingMark, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            // new IntakeCommand
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueRightIntermediaryFar, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueRightIntermediaryNear, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueGridRightRobotLeft, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading)
            // new ScoreGameObjectCommand
        );
    }        
}
