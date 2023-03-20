package frc.robot.commands.auto.test;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;


public class Test3M extends SequentialCommandGroup{
    public Test3M(Drivetrain drivetrain) {

        final PathPlannerTrajectory traj = PathPlanner.loadPath("Test3M", new PathConstraints(Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.5));

        addCommands(
            
            new FollowTrajectoryCommand(traj, true, drivetrain::getPose, drivetrain)
        );
    }        
}
