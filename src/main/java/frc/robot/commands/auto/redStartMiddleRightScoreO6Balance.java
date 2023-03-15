package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointConeNoWait;
import frc.robot.commands.auto.aboveChassis.ArmAutoSetpointWithEndingCone;
import frc.robot.commands.auto.common.ScoreConeHigh;
import frc.robot.commands.auto.drive.AutoCartesianVectorProfileToPointTargetCommand;
import frc.robot.commands.auto.drive.AutoEngagePP2PBack;
import frc.robot.commands.swerve.Lockdown;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class redStartMiddleRightScoreO6Balance extends SequentialCommandGroup{
    public redStartMiddleRightScoreO6Balance(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new ScoreConeHigh(elbow, intake, intakespinner, robotContainer , shoulder),
            new ArmAutoSetpointWithEndingCone(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, robotContainer),
            new ArmAutoSetpointConeNoWait(elbow, shoulder, intakespinner, Constants.Arm.ArmPosition.CARRY),
            new AutoCartesianVectorProfileToPointTargetCommand(
                Constants.Auto.redMobilityPoint, 
                drivetrain::getTranslation, 
                drivetrain::velocity,
                Constants.Auto.trajectoryTolerance,
                3, 
                Units.inchesToMeters(160),
                Units.inchesToMeters(60), 
                autoDrive, 
                drivetrain,
                heading
            ),
            new WaitCommand(1.5),
            new AutoEngagePP2PBack(
                Constants.Auto.redRightCenterOfChargeStation, 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(200), 
                Units.inchesToMeters(45), 
                drivetrain::getGyroscopePitch,
                autoDrive, 
                drivetrain,
                heading
            ),
            new Lockdown(autoDrive, drivetrain, heading).withTimeout(0.25)
        );
    }        
}
