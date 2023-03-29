package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private CANSparkMax intakeMotor = new CANSparkMax(17, MotorType.kBrushless);
    private SparkMaxLimitSwitch beamBreak = intakeMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    private SparkMaxAnalogSensor intakeSpinnerLamprey = intakeMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    private SparkMaxPIDController controller;
    private RelativeEncoder intakeEncoder;
    private double peakOutput = 1.0;
    
    

    public Intake() {
        beamBreak.enableLimitSwitch(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
        intakeMotor.setOpenLoopRampRate(peakOutput);
        intakeMotor.setSmartCurrentLimit(40, 40, 40); // TODO CHANGE LATER
        intakeMotor.setOpenLoopRampRate(0);
        intakeEncoder = intakeMotor.getEncoder();
        controller = intakeMotor.getPIDController();
        controller.setP(Constants.Arm.intakeP);
        controller.setI(Constants.Arm.intakeI);
        controller.setD(Constants.Arm.intakeD);
        controller.setOutputRange(-1, 1);
        controller.setFeedbackDevice(intakeEncoder);
    }

    public void setIntakeSpeed(double speed) {       
        intakeMotor.set(speed);
    }

    public double getIntakeMotorTemperature() {
        return intakeMotor.getMotorTemperature();
    }

    public boolean hasCone() {
        return beamBreak.isPressed();
    }

     public double getIntakeSpinnerLampreyVoltage() {
         return intakeSpinnerLamprey.getVoltage();
    }
    
    public double getIntakePosition() {
       return intakeEncoder.getPosition();
    }
    
    public void holdPosition(double setpoint) {
        controller.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
    }

    public void log() {
        if (Constants.DashboardLogging.INTAKE) {
            SmartDashboard.putNumber("Arm/Intake motorTemp", getIntakeMotorTemperature());
            SmartDashboard.putNumber("Arm/Intake Lamprey Voltage", getIntakeSpinnerLampreyVoltage());
            SmartDashboard.putNumber("Arm/Intake Position", intakeEncoder.getPosition());
            SmartDashboard.putNumber("Arm/Intake Motor Speed", intakeMotor.getAppliedOutput());
        }
        SmartDashboard.putBoolean("Arm/Intake Beam Break", beamBreak.isPressed());
    }

}