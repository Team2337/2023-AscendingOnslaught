package frc.robot.commands.auto.pathplanner.blue.bump;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointCubeWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.common.BlueGBotScoreC8GMidScoreO8;
import frc.robot.commands.auto.common.ScoreConeHigh;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class blueRightyMidScoreC9GBotScoreC8GMidScoreO8 extends SequentialCommandGroup{
    public blueRightyMidScoreC9GBotScoreC8GMidScoreO8(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ScoreConeHigh(elbow, intake, intakespinner, robotContainer, shoulder),
            new BlueGBotScoreC8GMidScoreO8(autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer,shoulder),
            new SequentialCommandGroup(
                new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
                new ArmAutoSetpointCubeWait(1.0, elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY)   
            )
        );
    }
}
