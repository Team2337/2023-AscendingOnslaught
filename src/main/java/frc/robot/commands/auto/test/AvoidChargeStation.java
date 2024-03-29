package frc.robot.commands.auto.test;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;


public class AvoidChargeStation extends SequentialCommandGroup{

    PathPlannerTrajectory traj;
    
    public AvoidChargeStation(PathPlannerTrajectory traj, AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {

        this.traj = traj;

        addCommands(
            
            new FollowTrajectoryCommand(traj, true, drivetrain::getPose, autoDrive, drivetrain, heading)
        );
    }        
}
