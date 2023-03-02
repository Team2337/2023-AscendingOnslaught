package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.common.DriveToPickupCone1;
import frc.robot.commands.auto.common.DriveToScoreHigh1;
import frc.robot.commands.auto.common.ScoreConeHigh;
import frc.robot.commands.auto.common.ScoreCubeHigh;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class PartnerShowcase extends SequentialCommandGroup{
    public PartnerShowcase(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ScoreCubeHigh(elbow, intake, intakespinner, shoulder),
            new DriveToPickupCone1(Constants.Auto.partnerShowcasePickup, autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer, shoulder),
            new DriveToScoreHigh1(Constants.Auto.partnerShowcaseScore, Constants.Arm.ArmPosition.SCOREHIGH, autoDrive, drivetrain, elbow, heading, intake, intakespinner, shoulder),
            new ScoreConeHigh(elbow, intake, intakespinner, robotContainer, shoulder)
        );
    }        
}