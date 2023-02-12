package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class IntakeSpinnerLamprey extends PIDSubsystem {
    
    private TalonFX intakeSpinnerMotor = new TalonFX(50);
    private double lampreyVoltage = 3.306;
    private double fullRange = 360 / lampreyVoltage;
    private double offset = 0;
    private Supplier<Double> voltage;

    private double peakOutput = 0.1;
    private double tolerance = 1;
    private static double kP = 0.1;
    private static double kI = 0;
    private static double kD = 0;
    private double setpoint = 0;

    public IntakeSpinnerLamprey(Supplier<Double> voltage) {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(tolerance);
        this.voltage = voltage; 

        intakeSpinnerMotor.configFactoryDefault();
        intakeSpinnerMotor.configForwardSoftLimitThreshold(0);
        intakeSpinnerMotor.configForwardSoftLimitEnable(false);
        intakeSpinnerMotor.configReverseSoftLimitThreshold(150000);
        intakeSpinnerMotor.configReverseSoftLimitEnable(false);
        intakeSpinnerMotor.setNeutralMode(NeutralMode.Brake);
        intakeSpinnerMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit());
        intakeSpinnerMotor.configPeakOutputForward(peakOutput, 10);
        intakeSpinnerMotor.configPeakOutputReverse(-peakOutput, 10);
        intakeSpinnerMotor.setInverted(TalonFXInvertType.Clockwise);
    }

      @Override
    protected void useOutput(double output, double setpoint) {
        setIntakeSpinnerMotorSpeed(output);
    }

    @Override
    protected double getMeasurement() {
        return getEncoderDegrees();
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void setIntakeSpinnerMotorSpeed(double speed) {       
        intakeSpinnerMotor.set(ControlMode.PercentOutput, speed);
    }   
   
    public double getTemperature() {
        return intakeSpinnerMotor.getTemperature();
    }
    
    public double getEncoderDegrees() {
        return (voltage.get() * fullRange) + offset;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("encoder degrees", getEncoderDegrees());
    }

  
}
