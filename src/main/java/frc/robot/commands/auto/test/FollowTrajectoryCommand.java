package frc.robot.commands.auto.test;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectoryCommand extends SequentialCommandGroup {
    private Drivetrain drivetrain;
        
    public FollowTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, Supplier<Pose2d> pose,  Drivetrain drivetrain ) {
        this.drivetrain = drivetrain;

        addCommands(
        new InstantCommand(() -> {
            if(isFirstPath) {
                drivetrain.resetPosition(traj.getInitialHolonomicPose());
            }
        }
        ),
        new PPSwerveControllerCommand(
            traj, 
            pose, // Pose supplier
            drivetrain.getKinematics(), // SwerveDriveKinematics
            new PIDController(.25, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(.25, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            drivetrain::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drivetrain // Requires this drive subsystem
        )
    );
}
}


