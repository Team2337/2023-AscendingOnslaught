package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class MoveForwardTest extends SequentialCommandGroup{
    public MoveForwardTest(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
        addCommands(
            new WaitCommand(1),
            new CartesianProfiledPointToPointCommand(
                new Translation2d(0, 3), 
                drivetrain::getTranslation, 
                drivetrain::getRotation, 
                3, 
                3, 
                Units.inchesToMeters(30), 
                Units.inchesToMeters(30), 
                autoDrive, 
                heading
                )
        );
    }        
}
