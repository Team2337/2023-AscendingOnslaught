package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class IntakeSpinnerTalon extends SubsystemBase {
    
    private TalonFX intakeSpinnerMotor = new TalonFX(33);
    private double peakOutput = 0.1;
    private double tolerance = 10000;
    private double kP = 0.1;
    private double kI = 0;
    private double kD = 0;
    private double setpoint = 0;

    public IntakeSpinnerTalon() {

        intakeSpinnerMotor.configFactoryDefault();
        intakeSpinnerMotor.config_kP(0, kP);
        intakeSpinnerMotor.config_kI(0, kI);
        intakeSpinnerMotor.config_kD(0, kD);
        intakeSpinnerMotor.setSelectedSensorPosition(0);
        intakeSpinnerMotor.configAllowableClosedloopError(0, tolerance, 10);
        intakeSpinnerMotor.configForwardSoftLimitThreshold(0);
        intakeSpinnerMotor.configForwardSoftLimitEnable(true);
        intakeSpinnerMotor.configReverseSoftLimitThreshold(150000);
        intakeSpinnerMotor.configReverseSoftLimitEnable(true);
        intakeSpinnerMotor.setNeutralMode(NeutralMode.Brake);
        intakeSpinnerMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit());
        intakeSpinnerMotor.configPeakOutputForward(peakOutput, 10);
        intakeSpinnerMotor.configPeakOutputReverse(-peakOutput, 10);
        
        intakeSpinnerMotor.setInverted(TalonFXInvertType.Clockwise);
    }

    public void setIntakeSpinnerMotorSpeed(double speed) {       
        intakeSpinnerMotor.set(ControlMode.PercentOutput, speed);
    }   
    public double getTemperature() {
        return intakeSpinnerMotor.getTemperature();
    }
    
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
    
    @Override
    public void periodic() {
        intakeSpinnerMotor.set(ControlMode.Position, setpoint);
    }

}
