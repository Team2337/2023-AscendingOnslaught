package frc.robot.commands.auto.pathplanner.blue.balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.aboveChassis.IntakeReverseCube;
import frc.robot.commands.auto.common.BlueBalanceFront;
import frc.robot.commands.auto.common.BlueLeftyLeftGToppyScoreC2GTopScoreO2;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.IntakeSpinnerLamprey;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class blueStartLeftyMidScoreW2GToppyScoreC2GTopScoreO2Balance extends SequentialCommandGroup{
    public blueStartLeftyMidScoreW2GToppyScoreC2GTopScoreO2Balance(AutoDrive autoDrive, Drivetrain drivetrain, Elbow elbow, Heading heading, Intake intake, IntakeSpinnerLamprey intakespinner, RobotContainer robotContainer, Shoulder shoulder) {
        addCommands(
            new IntakeReverseCube(intake).withTimeout(0.2),
            new BlueLeftyLeftGToppyScoreC2GTopScoreO2(autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer, shoulder),
            new BlueBalanceFront(autoDrive, drivetrain, elbow, heading, intake, intakespinner, robotContainer,shoulder)  
        );    
    }
}
