package frc.robot.subsystems.hardware;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase  {
  
  private PhotonCamera camera = new PhotonCamera("intake");
  private PhotonPipelineResult targets;
  private PhotonTrackedTarget ball;
  private double yawValue = 0.0;
  private boolean hasTarget = false;
  private List<TargetCorner> corners;
  private Double ballX;

  public PhotonVision() {
  }

  @Override
  public void periodic() {
    targets = camera.getLatestResult();
    if (targets.hasTargets()) {
      ball = targets.getBestTarget();
      yawValue = ball.getYaw();
      corners = ball.getDetectedCorners();
      hasTarget = true;
      ballX = getXValue(corners);
      SmartDashboard.putNumber("Photon X", ballX);
    } else {
      hasTarget = false;
      ballX = null;
    }
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public void changePipeline(int pipeline) {
    camera.setPipelineIndex(pipeline);
  }

  public double getYaw() {
    return yawValue;
  }

  public List<TargetCorner> getCorners() {
    return corners;
  }

  public double getFrameWidth() {
    //Find based off of PhotonVision output settings
    return 320;
  }

  public double getFrameCenter() {
    return getFrameWidth() / 2;
  }

  public Double getBallXValue() {
    return ballX;
  }

  private double getXValue(List<TargetCorner> targetCorners) {
    /**
     * PixyCam uses x-y coordinates to find balls and strafe, but PhotonVision only gives us rotations around axes for
     * finding our ball. However, getting the corner gives us a bounding box, which can be presumed to be pixels from
     * the camera output. So if we have a rectangle (the bounding box), we can get the x-values and divide them by two
     * to get the midpoint, and turn this midpoint of the ball to be relative to the camera center so that it can be
     * plugged into the PixyPickupCommand to run correctly.
     */


    //Hold Point values
    return (
      targetCorners.get(0) == targetCorners.get(1) ?          //Check if the first two are equal. There are only two unique values
      (targetCorners.get(0).x + targetCorners.get(2).x) / 2 : //If so, the first and third can be presumed to be unique. Use those
      (targetCorners.get(0).x + targetCorners.get(1).x) / 2   //Otherwise, resort to using the first two, which are different.
    );//If you don't like ternary, I can switch this out for a more readable if-statement
    
  }
}
