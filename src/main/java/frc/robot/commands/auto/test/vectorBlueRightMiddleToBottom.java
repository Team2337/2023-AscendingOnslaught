package frc.robot.commands.auto.test;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.drive.CartesianVectorProfileToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class vectorBlueRightMiddleToBottom extends SequentialCommandGroup{
    public vectorBlueRightMiddleToBottom(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(2),
            new CartesianVectorProfileToPointCommand(
                Constants.Auto.blueBottomStagingMark, 
                drivetrain::getTranslation,
                1.5,
                Units.inchesToMeters(80),
                autoDrive, 
                heading
            ),
            new WaitCommand(1),
            new CartesianVectorProfileToPointCommand(
                Constants.Auto.blueGridRightRobotCenter,
                drivetrain::getTranslation,
                1.5,
                Units.inchesToMeters(80),
                autoDrive,
                heading
            )
        );
    }        
}
