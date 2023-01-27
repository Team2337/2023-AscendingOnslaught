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
    private double elbowOffest = 0;
    private double mathAngle;

    public ArmJoystickCommand(Arm arm, XboxController joystick) {
        this.arm = arm;
        this.joystick = joystick;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        shouldHoldArm = true;
        
    }


    @Override
    public void execute() {
        var outputXAxis = Utilities.deadband(joystick.getLeftX(), 0.1);
        var outputYAxis = Utilities.deadband(joystick.getLeftY(), 0.1);
        if ((outputXAxis == 0) && (outputYAxis == 0)){ 
            if (shouldHoldArm) {
                arm.holdElbowPosition(arm.getElbowPositionTicks());
                arm.holdShoulderPosition(arm.getShoulderPositionTicks());
                shouldHoldArm = false;
            }
        } else {
            mathAngle = Units.radiansToDegrees(arm.convertTicksToAngles(arm.getShoulderPositionTicks())) + shoulderOffset;
            if (Utilities.deadband(joystick.getLeftX(), 0.1) == 0){
                deltaX = 0;
            }
            else {
                deltaX = joystick.getLeftX() * 1;
            }
            if (Utilities.deadband(joystick.getLeftY(), 0.1) == 0){
                deltaY = 0;
            }
            else {
                deltaY = -joystick.getLeftY() * 1;
            }
    
            if(Utilities.deadband(joystick.getLeftY(), 0.1) == 0 || Utilities.deadband(joystick.getLeftX(), 0.1) == 0) {
                currentX = Constants.SHOULDER_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathAngle)) + Constants.ELBOW_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathAngle) + arm.convertTicksToAngles(arm.getElbowPositionTicks()));
                currentY = Constants.SHOULDER_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathAngle)) + Constants.ELBOW_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathAngle) + arm.convertTicksToAngles(arm.getElbowPositionTicks()));
            }
            
    
            double targetX = currentX + deltaX;
            double targetY = currentY + deltaY;
            SmartDashboard.putNumber("A/Target X", targetX);
            SmartDashboard.putNumber("A/Target Y", targetY);
    
            double hypot = Math.sqrt((targetX * targetX) + (targetY * targetY));
            double theta_S2 = Math.acos((Math.pow(Constants.SHOULDER_ARM_LENGTH, 2) + Math.pow(hypot, 2) - Math.pow(Constants.ELBOW_ARM_LENGTH,2)) / (2.0 * hypot * Constants.ELBOW_ARM_LENGTH));
            double theta_S1 = Math.asin(targetY/hypot);
            double theta_E = Math.acos((Math.pow(Constants.SHOULDER_ARM_LENGTH, 2) + Math.pow(Constants.ELBOW_ARM_LENGTH, 2) - Math.pow(hypot,2)) / (2.0 * Constants.SHOULDER_ARM_LENGTH * Constants.ELBOW_ARM_LENGTH));
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

            //arm.holdElbowPosition(arm.convertAnglestoTicks(topSetpoint + bottomSetpoint));
            ///arm.holdShoulderPosition(arm.convertAnglestoTicks(bottomSetpoint));
        }

            SmartDashboard.putNumber("A/Top Angle (Degrees)", topSetpoint);
            SmartDashboard.putNumber("A/Bottom Angle (Degrees)", bottomSetpoint);

            SmartDashboard.putNumber("A/Shoulder Angle Math", mathAngle);

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
