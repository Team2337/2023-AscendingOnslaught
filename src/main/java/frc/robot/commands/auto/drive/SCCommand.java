package frc.robot.commands.auto.drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

public class SCCommand extends SwerveControllerCommand{
    private Supplier<Pose2d> poseSupplier;
    
    public SCCommand(Trajectory trajectory, Supplier<Pose2d> poseSupplier, SwerveDriveKinematics kinematics, PIDController xController, PIDController yController, ProfiledPIDController thetaController, Consumer<SwerveModuleState[]> outputModuleStates, Drivetrain drivetrain) {
        super(
        trajectory, 
        poseSupplier, 
        kinematics, 
        xController, 
        yController, 
        thetaController, 
        outputModuleStates, 
        drivetrain);
    }
}
