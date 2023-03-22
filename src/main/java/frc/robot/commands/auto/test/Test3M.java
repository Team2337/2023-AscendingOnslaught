package frc.robot.commands.auto.test;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;


public class Test3M extends SequentialCommandGroup{
    public Test3M(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {

        final PathPlannerTrajectory traj = PathPlanner.loadPath("Test3M", new PathConstraints(5.16, 2.0));

        addCommands(
            
            new FollowTrajectoryCommand(traj, true, drivetrain::getPose, autoDrive, drivetrain, heading)
        );
    }        
}
