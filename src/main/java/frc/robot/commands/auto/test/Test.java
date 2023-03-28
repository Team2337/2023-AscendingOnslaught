package frc.robot.commands.auto.test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.common.ScoreConeHigh;
import frc.robot.commands.auto.common.ScoreConeMid;
import frc.robot.commands.auto.drive.CartesianVectorProfileToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class Test extends SequentialCommandGroup{
    public Test(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading, Elbow elbow, Intake intake, IntakeSpinnerLamprey intakeSpinner, Shoulder shoulder, RobotContainer robotContainer) {
        addCommands(
            new ScoreConeHigh(elbow, intake, intakeSpinner, robotContainer, shoulder)
           /* new CartesianVectorProfileToPointCommand(
                new Translation2d(2, 0), 
                drivetrain::getTranslation,
                1.5,
                Units.inchesToMeters(80),
                autoDrive, 
                heading
            )*/
        );
    }        
}
