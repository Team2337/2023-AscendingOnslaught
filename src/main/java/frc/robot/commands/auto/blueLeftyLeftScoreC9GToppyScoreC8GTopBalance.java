package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class blueLeftyLeftScoreC9GToppyScoreC8GTopBalance extends SequentialCommandGroup{
    public blueLeftyLeftScoreC9GToppyScoreC8GTopBalance(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(2),
            // new ScoreGameObjectCommand
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueToppyTopStagingMark, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            // new IntakeCommand
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueGridLeftRobotCenter, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            // new ScoreGameObjectCommand
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueLeftIntermediaryFar, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueTopStagingMark, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading),
            // new IntakeCommand
            new CartesianVectorProfileToPointCommand(Constants.Auto.blueLeftCenterOfChargeStation, drivetrain::getTranslation, 1.5, Units.inchesToMeters(162), Units.inchesToMeters(80), autoDrive, heading)
        );
    }        
}
