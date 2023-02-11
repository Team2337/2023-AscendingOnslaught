package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Shoulder;

public class ArmJoystickCommand extends CommandBase {
    //private Arm arm;
    private Shoulder shoulder;
    private Elbow elbow;
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
    private double ticksPerDegree = 1094.44;
    Supplier<Boolean>lampreyBroken;
    double testx = 0;
    double testy = 0;
    public ArmJoystickCommand(Elbow elbow, Shoulder shoulder, XboxController joystick, Supplier<Boolean>lampreyBroken) {
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.joystick = joystick;
        this.lampreyBroken = lampreyBroken;
        addRequirements(elbow, shoulder);
    }

    @Override
    public void initialize() {
        //TODO:  can we change this to something that makes sense when you read it, such as needCurrentSetpoint? or switch logic so boolean reads haveCurrentSetpoint.
        //shouldHoldArm = true;


        

        
    }


    @Override
    public void execute() {
        if (lampreyBroken.get()){
            shoulder.disable();
            elbow.disable();
            double outputShoulder = Utilities.deadbandAndSquare(-joystick.getLeftY(), 0.15);
            double outputElbow = Utilities.deadbandAndSquare(joystick.getRightY(), 0.15);
            if ((outputShoulder == 0) && (outputElbow == 0)){ 
                if (shouldHoldArm) {
                    elbow.holdElbowPosition(elbow.getElbowPositionTicks());
                    shoulder.holdShoulderPosition(shoulder.getShoulderPositionTicks());
                    shouldHoldArm = false;
                }
            } else {
                elbow.setElbowSpeed(outputElbow);
                shoulder.setShoulderSpeed(outputShoulder);
                shouldHoldArm = true;
            }
    }
        else {
            shoulder.enable();
            elbow.enable();

        
        // Convert sensor readings to angles as used in our forward and inverse kinematics.
        // Shoulder angle shoud be zero when level with the ground and pointing straight back from the robot (when back of the robot to the right, angles are positive CCW)
        mathShoulderAngle = shoulder.getShoulderLampreyDegrees();
        // Elbow Angle is zero when parallel with first/bottom arm (when back of the robot to the right, angles are positive CCW)
        mathElbowAngle = elbow.getElbowLampreyDegrees();
        // Calculate the current X,Y location of the intake
        currentX = Constants.Arm.SHOULDER_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle)) + Constants.Arm.ELBOW_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle) + Units.degreesToRadians(mathElbowAngle));
        currentY = Constants.Arm.SHOULDER_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathShoulderAngle)) + Constants.Arm.ELBOW_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathShoulderAngle) + Units.degreesToRadians(mathElbowAngle));

        
        //Read Joystick inputs and apply a deadband
        // TODO:  do we want X and Y on different joysticks??  variable for deadband?
        joystickX = Utilities.deadbandAndSquare(-joystick.getRightY(), 0.1);
        joystickY = Utilities.deadbandAndSquare(-joystick.getLeftY(), 0.1);

        deltaX = joystickX * joystickP; 
        deltaY = joystickY * joystickP;
        //deltaX = 0;
        //deltaY = 0;
        if (currentX > 0) {
            if (currentX < 18.0 && deltaX < 0) {
                deltaX = 0;
            }
            if (Math.sqrt(Math.pow(currentX,2) +Math.pow(currentY, 2)) > ((Constants.Arm.ELBOW_ARM_LENGTH + Constants.Arm.SHOULDER_ARM_LENGTH) * .9)) {
                if (deltaX > 0) {
                    deltaX = 0;
                }
                if (deltaY > 0) {
                    deltaY = 0;
                }
            }

        } else {
            if (currentX > -18.0 && deltaX > 0) {
                deltaX = 0;
            }
            if (Math.sqrt(Math.pow(currentX,2) +Math.pow(currentY, 2)) > ((Constants.Arm.ELBOW_ARM_LENGTH + Constants.Arm.SHOULDER_ARM_LENGTH) * .9)) {
                if (deltaX < 0) {
                    deltaX = 0;
                }
                if (deltaY > 0) {
                    deltaY = 0;
                }
            }
        }

        // Adjust the target X,Y location of the intake based on joystick inputs
        double targetX = currentX + deltaX;
        double targetY = currentY + deltaY;
        //targetX = -33;
        //targetY = 25;
        SmartDashboard.putNumber("Arm K/DeltaX", deltaX);
        SmartDashboard.putNumber("Arm K/DeltaY", deltaY);
        SmartDashboard.putNumber("Arm K/Target X", targetX);
        SmartDashboard.putNumber("Arm K/Target Y", targetY);
        
        // Calculate new arm angles based on target X,Y    
        double hypot = Math.sqrt((targetX * targetX) + (targetY * targetY));
        double theta_S2 = Math.acos((Math.pow(Constants.Arm.SHOULDER_ARM_LENGTH, 2) + Math.pow(hypot, 2) - Math.pow(Constants.Arm.ELBOW_ARM_LENGTH,2)) 
        / (2.0 * hypot * Constants.Arm.SHOULDER_ARM_LENGTH));
       // double theta_S1 = Math.asin(targetY/hypot);
       double theta_S1 = Math.atan2(targetY, targetX);
        double theta_E = Math.acos((Math.pow(Constants.Arm.SHOULDER_ARM_LENGTH, 2) + Math.pow(Constants.Arm.ELBOW_ARM_LENGTH, 2) - Math.pow(hypot,2)) 
        / (2.0 * Constants.Arm.SHOULDER_ARM_LENGTH * Constants.Arm.ELBOW_ARM_LENGTH));
        SmartDashboard.putNumber("Arm K/hypot", hypot);
        SmartDashboard.putNumber("Arm K/theta_S2", Units.radiansToDegrees(theta_S2));
        SmartDashboard.putNumber("Arm K/theta_S1", Units.radiansToDegrees(theta_S1));
        SmartDashboard.putNumber("Arm K/theta_E", Units.radiansToDegrees(theta_E));

        // TODO: NOT SURE ON LOGIC, HAVE TO WALK THRU AGAIN
        // Final steps to determine new angle setpoints differs based on the quadrant (x,y) is in.
       /*  if (targetY >= 0) {
            bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1 + theta_S2));
        }
        else {
            bottomSetpoint = (int)(Units.radiansToDegrees(theta_S2 + theta_S1));
        } */
        if (targetX < 0) {
            topSetpoint = (int)(Units.radiansToDegrees(Math.PI - theta_E));
            if (targetY < 0){
                bottomSetpoint = (int)(Units.radiansToDegrees((2*Math.PI) + theta_S1-theta_S2));
            }
            else {
                bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1-theta_S2));
            }
            
            }
        else {
                topSetpoint = (int)(Units.radiansToDegrees(theta_E-Math.PI));
                bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1 + theta_S2));
        }
            
                /*else {
            topSetpoint = (int)(Units.radiansToDegrees(theta_E - Math.PI));
        } */

        //TODO: THINK WE NEED TO SUBTRACT OUT OFFSETS BEFORE SETTING POSITIONS
        double shoulderTarget = bottomSetpoint;
        double elbowTarget = topSetpoint;

        //elbowTarget = SmartDashboard.getNumber("A/test elbo target", elbowTarget);
        //shoulderTarget = SmartDashboard.getNumber("A/test shoulder target", shoulderTarget);




        if ((joystickX == 0) && (joystickY == 0)){ 
            if (shouldHoldArm) {
                shoulder.setSetpoint(shoulder.getShoulderLampreyDegrees());
                elbow.setSetpoint(elbow.getElbowLampreyDegrees());
                shouldHoldArm = false;
            }
        } else {
            // Change boolean so that if joysticks go back to zero, we will get position/set setpoint, stoping the arms movement.
            shouldHoldArm = true;    

            shoulder.setSetpoint(shoulderTarget);
            elbow.setSetpoint(elbowTarget);
        }
            SmartDashboard.putNumber("Arm K/shoulderTarget", shoulderTarget);
            SmartDashboard.putNumber("Arm K/elbowTarget", elbowTarget);
            SmartDashboard.putNumber("Arm K/Math Top Angle (Degrees)", topSetpoint);
            SmartDashboard.putNumber("Arm K/Math Bottom Angle (Degrees)", bottomSetpoint);

            SmartDashboard.putNumber("Arm K/Actual Shoulder Angle (Degrees)", shoulder.getShoulderLampreyDegrees());
            SmartDashboard.putNumber("Arm K/Actual Elbow Angle (Degrees)", elbow.getElbowLampreyDegrees());
            SmartDashboard.putNumber("Arm K/Shoulder Angle Math", mathShoulderAngle);
            SmartDashboard.putNumber("Arm K/Elbow Angle Math", mathElbowAngle);

            SmartDashboard.putNumber("Arm K/Current X", currentX);
            SmartDashboard.putNumber("Arm K/Current Y", currentY);
    
            SmartDashboard.putNumber("Arm K/Elbow Predicted Degrees", elbowTarget);
            SmartDashboard.putNumber("Arm K/Bottom Predicted Ticks", shoulderTarget);
    }
    }


        


    @Override
    public boolean isFinished() {
        return false;
    }

    
}
