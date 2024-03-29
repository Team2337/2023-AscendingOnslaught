package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.common.DriveToPickupCone1Test;
import frc.robot.commands.auto.common.ScoreCubeMid;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class armTest extends SequentialCommandGroup{
    public armTest(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ScoreCubeMid(elbow, intake, intakespinner, robotContainer , shoulder),
            new DriveToPickupCone1Test(Constants.Auto.blueBottom8, autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer, shoulder)
        );
    }        
}
