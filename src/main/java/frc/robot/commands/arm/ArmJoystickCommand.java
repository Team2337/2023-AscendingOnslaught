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
    private double joystickX = 0;
    private double joystickY = 0;
    private double joystickP = 2.5;//Not actually used in a PID
    private double currentX = 0;
    private double currentY = 0;
    private double deltaX = 0;
    private double deltaY = 0;
    private double bottomSetpoint = 0;
    private double topSetpoint = 0;
    private boolean shouldHoldArm = true;
    private double shoulderOffset = 0;
    private double elbowOffset = 0;
    private double mathShoulderAngle, mathElbowAngle;
    private double ticksPerDegree = 426.667;
    double testx = 0;
    double testy = 0;
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
        // Elbow Angle is zero when parallel with first/bottom arm (when back of the robot to the right, angles are positive CCW)
        mathElbowAngle = Units.radiansToDegrees(arm.convertTicksToRadians(arm.getElbowPositionTicks())) - mathShoulderAngle + elbowOffset;
        // Calculate the current X,Y location of the intake
        currentX = Constants.SHOULDER_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle)) + Constants.ELBOW_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle) + Units.degreesToRadians(mathElbowAngle));
        currentY = Constants.SHOULDER_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathShoulderAngle)) + Constants.ELBOW_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathShoulderAngle) + Units.degreesToRadians(mathElbowAngle));

        
        //Read Joystick inputs and apply a deadband
        // TODO:  do we want X and Y on different joysticks??  variable for deadband?
        joystickX = Utilities.deadband(-joystick.getRightY(), 0.1);
        joystickY = Utilities.deadband(-joystick.getLeftY(), 0.1);

        deltaX = joystickX * joystickP; 
        deltaY = joystickY * joystickP;
        //deltaX = 0;
        //deltaY = 0;

        // Adjust the target X,Y location of the intake based on joystick inputs
        double targetX = currentX + deltaX;
        double targetY = currentY + deltaY;
        //targetX = -33;
        //targetY = 25;
        SmartDashboard.putNumber("A/DeltaX", deltaX);
        SmartDashboard.putNumber("A/DeltaY", deltaY);
        SmartDashboard.putNumber("A/Target X", targetX);
        SmartDashboard.putNumber("A/Target Y", targetY);
        
        // Calculate new arm angles based on target X,Y    
        double hypot = Math.sqrt((targetX * targetX) + (targetY * targetY));
        double theta_S2 = Math.acos((Math.pow(Constants.SHOULDER_ARM_LENGTH, 2) + Math.pow(hypot, 2) - Math.pow(Constants.ELBOW_ARM_LENGTH,2)) / (2.0 * hypot * Constants.ELBOW_ARM_LENGTH));
        double theta_S1 = Math.asin(targetY/hypot);
        double theta_E = Math.acos((Math.pow(Constants.SHOULDER_ARM_LENGTH, 2) + Math.pow(Constants.ELBOW_ARM_LENGTH, 2) - Math.pow(hypot,2)) / (2.0 * Constants.SHOULDER_ARM_LENGTH * Constants.ELBOW_ARM_LENGTH));
        SmartDashboard.putNumber("A/hypot", hypot);
        SmartDashboard.putNumber("A/theta_S2", Units.radiansToDegrees(theta_S2));
        SmartDashboard.putNumber("A/theta_S1", Units.radiansToDegrees(theta_S1));
        SmartDashboard.putNumber("A/theta_E", Units.radiansToDegrees(theta_E));

        // TODO: NOT SURE ON LOGIC, HAVE TO WALK THRU AGAIN
        // Final steps to determine new angle setpoints differs based on the quadrant (x,y) is in.
        if (targetY >= 0) {
            bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1 + theta_S2));
        }
        else {
            bottomSetpoint = (int)(Units.radiansToDegrees(theta_S2 + theta_S1));
        }
        if (targetX < 0) {
            topSetpoint = (int)(Units.radiansToDegrees(Math.PI - theta_E));
            if (targetY < 0) {
                bottomSetpoint = (int)(Units.radiansToDegrees(Math.PI - (theta_S2 - theta_S1)));
            }
            else {
                bottomSetpoint = (int)(Units.radiansToDegrees(Math.PI - (theta_S1 + theta_S2)));
            }
            
        }
        else {
            topSetpoint = (int)(Units.radiansToDegrees(theta_E - Math.PI));
        } 

        //TODO: THINK WE NEED TO SUBTRACT OUT OFFSETS BEFORE SETTING POSITIONS
        double shoulderTarget = bottomSetpoint - shoulderOffset;
        double elbowTarget = (topSetpoint - elbowOffset) + shoulderTarget;

        //elbowTarget = SmartDashboard.getNumber("A/test elbo target", elbowTarget);
        //shoulderTarget = SmartDashboard.getNumber("A/test shoulder target", shoulderTarget);




        if ((joystickX == 0) && (joystickY == 0)){ 
            if (shouldHoldArm) {
                arm.holdElbowPosition(arm.getElbowPositionTicks());
                arm.holdShoulderPosition(arm.getShoulderPositionTicks());
                shouldHoldArm = false;
            }
        } else {
            // Change boolean so that if joysticks go back to zero, we will get position/set setpoint, stoping the arms movement.
            shouldHoldArm = true;    

            arm.holdElbowPosition(arm.convertAnglestoTicks(elbowTarget));
            arm.holdShoulderPosition(arm.convertAnglestoTicks(shoulderTarget));
        }
            SmartDashboard.putNumber("A/shoulderTarget", shoulderTarget);
            SmartDashboard.putNumber("A/elbowTarget", elbowTarget);
            SmartDashboard.putNumber("A/Math Top Angle (Degrees)", topSetpoint);
            SmartDashboard.putNumber("A/Math Bottom Angle (Degrees)", bottomSetpoint);

            SmartDashboard.putNumber("A/Actual Shoulder Angle (Degrees)", Units.radiansToDegrees(arm.convertTicksToRadians(arm.getShoulderPositionTicks())));
            SmartDashboard.putNumber("A/Actual Elbow Angle (Degrees)", Units.radiansToDegrees(arm.convertTicksToRadians(arm.getElbowPositionTicks())));
            SmartDashboard.putNumber("A/Shoulder Angle Math", mathShoulderAngle);
            SmartDashboard.putNumber("A/Elbow Angle Math", mathElbowAngle);

            SmartDashboard.putNumber("A/Current X", currentX);
            SmartDashboard.putNumber("A/Current Y", currentY);
    
            SmartDashboard.putNumber("A/Elbow Predicted Ticks", arm.convertAnglestoTicks((elbowTarget)));
            SmartDashboard.putNumber("A/Bottom Predicted Ticks", arm.convertAnglestoTicks(shoulderTarget));
    }


        


    @Override
    public boolean isFinished() {
        return false;
    }

    
}
