package frc.robot.subsystems.wakaWaka;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class wakaWakaIntake extends SubsystemBase {
    
    private TalonFX intakeMotor = new TalonFX(68);
    private SparkMaxPIDController controller;
    private double peakOutput = 1.0;
    
    

    public wakaWakaIntake() {
        intakeMotor.configFactoryDefault();
        intakeMotor.configForwardSoftLimitThreshold(0);
        intakeMotor.configForwardSoftLimitEnable(false);
        intakeMotor.configReverseSoftLimitThreshold(150000);
        intakeMotor.configReverseSoftLimitEnable(false);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit());
        intakeMotor.configPeakOutputForward(peakOutput, 10);
        intakeMotor.configPeakOutputReverse(-peakOutput, 10);
        intakeMotor.setInverted(TalonFXInvertType.Clockwise);
        intakeMotor.setSelectedSensorPosition(0);
        
    }

    public void setIntakeSpeed(double speed) {       
        intakeMotor.set(ControlMode.PercentOutput,speed);
    }

    public double getIntakeMotorTemperature() {
        return intakeMotor.getTemperature();
    }
    
    public double getIntakePosition() {
       return intakeMotor.getSelectedSensorPosition();
    }
    
    public void holdPosition(double setpoint) {
        controller.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        log();
    }

    public void log() {
    }

}