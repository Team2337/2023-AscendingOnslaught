package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.drive.AutoEngagePP2P;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class blueStartMiddleMiddleBalance extends SequentialCommandGroup{
    public blueStartMiddleMiddleBalance(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(1),
            new AutoEngagePP2P(
                Constants.Auto.blueCenterOfChargeStation, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(60), 
                Units.inchesToMeters(60), 
                drivetrain::getGyroscopePitch,
                autoDrive, 
                drivetrain,
                heading
                )
        );
    }        
}
