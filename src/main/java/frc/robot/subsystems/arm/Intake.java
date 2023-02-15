package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private CANSparkMax intakeMotor = new CANSparkMax(17, MotorType.kBrushless);
    private SparkMaxLimitSwitch beamBreak = intakeMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    private SparkMaxAnalogSensor intakeSpinnerLamprey = intakeMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    private double peakOutput = 0.66;
    

    public Intake() {
        beamBreak.enableLimitSwitch(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
        intakeMotor.setOpenLoopRampRate(peakOutput);
        intakeMotor.setSmartCurrentLimit(40, 40, 40); // TODO CHANGE LATER
    }

    public Command RunIntake() {
        return new StartEndCommand(()->setIntakeSpeed(peakOutput), ()->setIntakeSpeed(0), this);
        
    }

    public Command RunOuttake() {
        return new StartEndCommand(()->setIntakeSpeed(-peakOutput), ()->setIntakeSpeed(0), this);

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

    @Override
    public void periodic() {
     SmartDashboard.putNumber("Intake/motorTemp", getIntakeMotorTemperature());
     SmartDashboard.putBoolean("Intake/Beam Break", beamBreak.isPressed());
    }

}