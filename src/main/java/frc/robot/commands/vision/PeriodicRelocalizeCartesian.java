package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.AllianceColor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LimelightColor;

public class PeriodicRelocalizeCartesian extends VisionCommand {

  private boolean disableRelocalizationInAuto = true;
  private int relocalizationCounterLimit = 5;
  private int relocalizationCounter = 0;
  private LimelightColor color;
  private boolean readLimelight;

  public PeriodicRelocalizeCartesian(Drivetrain drivetrain, Vision vision) {
    this.vision = vision;
    this.drivetrain = drivetrain;

    addRequirements(vision);
  }

  @Override
  public void execute() {
    //SmartDashboard.putBoolean("Reading Limelight", readLimelight);
    if(drivetrain.getPose().getX() < 6) {
      color = LimelightColor.ORANGE;
      readLimelight = true;
      //SmartDashboard.putString("We are...", "orange!");
    } else if (drivetrain.getPose().getX() > 8) {
      if (AllianceColor.getAllianceColor() == AllianceColor.Blue) {
        color = LimelightColor.PINK;
      } else {
        color = LimelightColor.BLUE;
      }
      readLimelight = true;
      //Override logic above to only look at the pink limelight. 
      color = LimelightColor.PINK;

      //SmartDashboard.putString("We are...", "blue!");
    } else {
      readLimelight = false;
    }

    // bypass which limelight logic for testing
    //TODO:  OMG remove this!
    color = LimelightColor.ORANGE;
      readLimelight = true;
    
    if (relocalizationCounter == relocalizationCounterLimit){
      //TODO: When do we turn this off?
      if ((DriverStation.isAutonomous() && disableRelocalizationInAuto) || !readLimelight) {
        return;
      }
      if (vision.getVisionPose(color).length != 0) {
        double visionPoseX = vision.getVisionPoseX(color);
        double visionPoseY = vision.getVisionPoseY(color);
        double visionRotation = vision.getVisionRotation(color);

        if (visionPoseX > 0 && visionPoseY > 0) {
          Pose2d pose = new Pose2d(
            new Translation2d(visionPoseX, visionPoseY),
            Rotation2d.fromDegrees(visionRotation)
          );
          //TODO: Check units here.  I think we are mixing ms and sec.
          drivetrain.addVisionMeasurement(pose, Timer.getFPGATimestamp() - ((Constants.Vision.IMAGE_PROCESSING_LATENCY_MS + vision.getLatency(color) + 2) / 1000));
        //System.out.println("counting " + relocalizationCounter + " " + visionPoseX + " " + visionPoseY + " " + visionRotation);
        }
      }
      relocalizationCounter = -1;
    }
    relocalizationCounter++; 
  }
}
