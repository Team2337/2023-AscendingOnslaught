package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants;

public class Intake extends PIDSubsystem {
    
    private CANSparkMax intakeMotor = new CANSparkMax(17, MotorType.kBrushless);
    private SparkMaxLimitSwitch beamBreak = intakeMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    private SparkMaxAnalogSensor intakeSpinnerLamprey = intakeMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    private PIDController controller;
    private RelativeEncoder intakeEncoder;
    private double peakOutput = 0.66;
    
    

    public Intake() {
        super(new PIDController(Constants.Arm.intakeP, Constants.Arm.intakeI, Constants.Arm.intakeD));
        beamBreak.enableLimitSwitch(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
        intakeMotor.setOpenLoopRampRate(peakOutput);
        intakeMotor.setSmartCurrentLimit(40, 40, 40); // TODO CHANGE LATER
        intakeEncoder = intakeMotor.getEncoder();
        controller.setTolerance(100);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        setIntakeSpeed(output);
    }
  
    @Override
    protected double getMeasurement() {
      return intakeEncoder.getPosition();
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

    public boolean atSetpoint() {
        return getController().atSetpoint();
      }
    
    public double getIntakePosition() {
       return intakeEncoder.getPosition();
    }
    
    public void holdPosition(double setpoint) {
        setSetpoint(setpoint);
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
            SmartDashboard.putNumber("Arm/Intake Get Setpoint", controller.getSetpoint());
            SmartDashboard.putNumber("Arm/Intake Motor Speed", intakeMotor.getAppliedOutput());

        }
        SmartDashboard.putBoolean("Arm/Intake Beam Break", beamBreak.isPressed());
    }

}