package frc.robot.subsystems.wakaWaka;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class wakaWakaIntake extends SubsystemBase {
    
    private TalonFX intakeMotor = new TalonFX(61);
    private double peakOutput = 0.5;
    
    

    public wakaWakaIntake() {
        intakeMotor.configFactoryDefault();
        intakeMotor.config_kP(0, 0.2);
        intakeMotor.config_kI(0, 0);
        intakeMotor.config_kD(0, 0);
        intakeMotor.configForwardSoftLimitThreshold(0);
        intakeMotor.configForwardSoftLimitEnable(false);
        intakeMotor.configReverseSoftLimitThreshold(150000);
        intakeMotor.configReverseSoftLimitEnable(false);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit());
        intakeMotor.configPeakOutputForward(peakOutput, 10);
        intakeMotor.configPeakOutputReverse(-peakOutput, 10);
        intakeMotor.setInverted(TalonFXInvertType.CounterClockwise);
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
    
    public void holdPosition() {
        intakeMotor.set(ControlMode.Position, intakeMotor.getSelectedSensorPosition());
    }

    @Override
    public void periodic() {
        log();
    }

    public void log() {
    }

}