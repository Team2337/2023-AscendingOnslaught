package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;

public class IntakeSpinnerLamprey extends PIDSubsystem {
    
    private TalonFX intakeSpinnerMotor = new TalonFX(50);
    private double lampreyVoltage = 2.26;
    private double fullRange = 360.0 / lampreyVoltage;
    private double offset = 0;
    private Supplier<Double> voltage;
    private double pastCurrent;
    private Supplier<GamePiece> gamePiece;

    private double peakOutput = 1.0;
    private double tolerance = 0.5;
    private static double kP = 0.01;
    private static double kI = 0;
    private static double kD = 0;

    public IntakeSpinnerLamprey(Supplier<Double> voltage, Supplier<GamePiece> gamePiece) {
        super(new PIDController(kP, kI, kD));
        this.voltage = voltage;
        this.gamePiece = gamePiece;

        intakeSpinnerMotor.configFactoryDefault();
        intakeSpinnerMotor.configForwardSoftLimitThreshold(0);
        intakeSpinnerMotor.configForwardSoftLimitEnable(false);
        intakeSpinnerMotor.configReverseSoftLimitThreshold(150000);
        intakeSpinnerMotor.configReverseSoftLimitEnable(false);
        intakeSpinnerMotor.setNeutralMode(NeutralMode.Brake);
        intakeSpinnerMotor.configStatorCurrentLimit(defaultCurrentLimit());
        intakeSpinnerMotor.configPeakOutputForward(peakOutput, 10);
        intakeSpinnerMotor.configPeakOutputReverse(-peakOutput, 10);
        intakeSpinnerMotor.setInverted(TalonFXInvertType.Clockwise);

        getController().setTolerance(tolerance);
        enable();
        setSetpoint(getEncoderDegrees());
    }

      @Override
    protected void useOutput(double output, double setpoint) {
    //    SmartDashboard.putNumber("Arm/Wrist PID Output", output);
        setIntakeSpinnerMotorSpeed(output);
    }

    @Override
    protected double getMeasurement() {
        return getEncoderDegrees();
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }
    public double getSetpoint() {
        return m_controller.getSetpoint();
    }

    public void setIntakeSpinnerMotorSpeed(double speed) {       
        intakeSpinnerMotor.set(ControlMode.PercentOutput, speed);
    }   

    public static StatorCurrentLimitConfiguration defaultCurrentLimit() {
        return new StatorCurrentLimitConfiguration(true, 10.0, 10.0, 1.0);
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
        log();
    }

    public void log() {
        if (Constants.DashboardLogging.INTAKESPINNER) {
            SmartDashboard.putNumber("Arm/ Wrist Setpoint", getSetpoint());
            SmartDashboard.putNumber("Arm/Wrist Output", intakeSpinnerMotor.getMotorOutputPercent());
            SmartDashboard.putString("Arm/Wrist Game Piece", gamePiece.get().toString());
            SmartDashboard.putNumber("Arm/Intake Spinner Current", intakeSpinnerMotor.getStatorCurrent());
        }
        SmartDashboard.putNumber("Arm/Wrist Degrees", getEncoderDegrees());
        
    }
  
}
