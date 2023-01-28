package frc.robot.commands.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Arm;

public class ArmJoystickCommand extends CommandBase {
    private Arm arm;
    private XboxController joystick;
    private double currentX;
    private double currentY;
    private double deltaX;
    private double deltaY;
    private double bottomSetpoint = 0;
    private double topSetpoint = 0;
    private boolean shouldHoldArm = true;
    private double shoulderOffset = 90;
    private double elbowOffset = 0;
    private double mathShoulderAngle, mathElbowAngle;

    public ArmJoystickCommand(Arm arm, XboxController joystick) {
        this.arm = arm;
        this.joystick = joystick;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        //TODO:  can we change this to something that makes sense when you read it, such as needCurrentSetpoint? or switch logic so boolean reads haveCurrentSetpoint.
        shouldHoldArm = true;
        
    }


    @Override
    public void execute() {

        // Convert sensor readings to angles as used in our forward and inverse kinematics.
        // Shoulder angle shoud be zero when level with the ground and pointing straight back from the robot (when back of the robot to the right, angles are positive CCW)
        mathShoulderAngle = Units.radiansToDegrees(arm.convertTicksToRadians(arm.getShoulderPositionTicks())) + shoulderOffset;
        // Elbo Angle is zero when parallel with first/bottom arm (when back of the robot to the right, angles are positive CCW)
        mathElbowAngle = Units.radiansToDegrees(arm.convertTicksToRadians(arm.getShoulderPositionTicks() + arm.getElbowPositionTicks())) + elbowOffset;

        //Read Joystick inputs and apply a deadband
        // TODO:  do we want X and Y on different joysticks??  variable for deadband?
        var outputXAxis = Utilities.deadband(joystick.getLeftX(), 0.1);
        var outputYAxis = Utilities.deadband(-joystick.getLeftY(), 0.1);

        if ((outputXAxis == 0) && (outputYAxis == 0)){ 
            if (shouldHoldArm) {
                arm.holdElbowPosition(arm.getElbowPositionTicks());
                arm.holdShoulderPosition(arm.getShoulderPositionTicks());
                shouldHoldArm = false;
            }
        } else {
            // Change boolean so that if joysticks go back to zero, we will get position/set setpoint, stoping the arms movement.
                shouldHoldArm = true;
                
                //TODO:  didn't we already deadband above
            if (Utilities.deadband(joystick.getLeftX(), 0.1) == 0){
                deltaX = 0;
            }
            else {
                deltaX = joystick.getLeftX() * 1; //TODO:  use variable
            }
               //TODO:  didn't we already deadband above
            if (Utilities.deadband(joystick.getLeftY(), 0.1) == 0){
                deltaY = 0;
            }
            else {
                deltaY = -joystick.getLeftY() * 1; //TODO:  use variable
            }
    
            // Calculate the current X,Y location of the intake
            currentX = Constants.SHOULDER_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle)) + Constants.ELBOW_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle) + arm.convertTicksToRadians(arm.getElbowPositionTicks()));
            currentY = Constants.SHOULDER_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathShoulderAngle)) + Constants.ELBOW_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathShoulderAngle) + arm.convertTicksToRadians(arm.getElbowPositionTicks()));
       
            // Adjust the target X,Y location of the intake based on joystick inputs
            double targetX = currentX + deltaX;
            double targetY = currentY + deltaY;
            SmartDashboard.putNumber("A/Target X", targetX);
            SmartDashboard.putNumber("A/Target Y", targetY);

            // Calculate new arm angles based on target X,Y    
            double hypot = Math.sqrt((targetX * targetX) + (targetY * targetY));
            double theta_S2 = Math.acos((Math.pow(Constants.SHOULDER_ARM_LENGTH, 2) + Math.pow(hypot, 2) - Math.pow(Constants.ELBOW_ARM_LENGTH,2)) / (2.0 * hypot * Constants.ELBOW_ARM_LENGTH));
            double theta_S1 = Math.asin(targetY/hypot);
            double theta_E = Math.acos((Math.pow(Constants.SHOULDER_ARM_LENGTH, 2) + Math.pow(Constants.ELBOW_ARM_LENGTH, 2) - Math.pow(hypot,2)) / (2.0 * Constants.SHOULDER_ARM_LENGTH * Constants.ELBOW_ARM_LENGTH));

            // TODO: NOT SURE ON LOGIC, HAVE TO WALK THRU AGAIN
            // Final steps to determine new angle setpoints differs based on the quadrant (x,y) is in.
            if (targetY >= 0) {
                bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1 + theta_S2));
            }
            else {
                bottomSetpoint = (int)(Units.radiansToDegrees(theta_S2 - theta_S1));
            }
            if (targetX < 0) {
                topSetpoint = (int)(Units.radiansToDegrees(Math.PI - theta_E));
                bottomSetpoint = (int)(Units.radiansToDegrees(Math.PI - (theta_S1 + theta_S2)));
            }
            else {
                topSetpoint = (int)(Units.radiansToDegrees(theta_E - Math.PI));
            } 

            //TODO: THINK WE NEED TO SUBTRACT OUT OFFSETS BEFORE SETTING POSITIONS

            //arm.holdElbowPosition(arm.convertAnglestoTicks(topSetpoint + bottomSetpoint));
            ///arm.holdShoulderPosition(arm.convertAnglestoTicks(bottomSetpoint));
        }

            SmartDashboard.putNumber("A/Top Angle (Degrees)", topSetpoint);
            SmartDashboard.putNumber("A/Bottom Angle (Degrees)", bottomSetpoint);

            SmartDashboard.putNumber("A/Shoulder Angle Math", mathShoulderAngle);
            SmartDashboard.putNumber("A/Elbow Angle Math", mathElbowAngle);

            SmartDashboard.putNumber("A/Current X", currentX);
            SmartDashboard.putNumber("A/Current Y", currentY);
    
            SmartDashboard.putNumber("A/Elbow Predicted Ticks", arm.convertAnglestoTicks(topSetpoint));
            SmartDashboard.putNumber("A/Bottom Predicted Ticks", arm.convertAnglestoTicks(bottomSetpoint));
    }


        


    @Override
    public boolean isFinished() {
        return false;
    }

    
}
