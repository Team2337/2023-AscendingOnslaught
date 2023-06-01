package frc.robot.subsystems.wakaWaka;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.nerdyfiles.utilities.CTREUtils;



public class wakaWakaArm extends PIDSubsystem {
    private TalonFX armMotor = new TalonFX(60);
    private DigitalInput armLimit = new DigitalInput(0);
    private static double kP = 0.0002;
    private static double kI = 0;
    private static double kD = 0;
    private double peakOutput = 1.0;


    public wakaWakaArm() {
        super(new PIDController(kP, kI, kD));
        armMotor.configFactoryDefault();
        armMotor.configForwardSoftLimitThreshold(30000);
        armMotor.configForwardSoftLimitEnable(false);
        armMotor.configReverseSoftLimitThreshold(0);
        armMotor.configReverseSoftLimitEnable(false);
        armMotor.setNeutralMode(NeutralMode.Coast);
        armMotor.configStatorCurrentLimit(CTREUtils.wakaCurrentLimit());
        armMotor.configPeakOutputForward(peakOutput, 10);
        armMotor.configPeakOutputReverse(-peakOutput, 10);
        armMotor.setInverted(TalonFXInvertType.Clockwise);
        armMotor.setSelectedSensorPosition(0);

        enable();
    }
    public void changeEncoderPosition(double setpoint) {
        armMotor.setSelectedSensorPosition(setpoint);
    }

    public boolean getArmLimit() {
        return !armLimit.get();
    }

    public void setSpeed(double speed) {
        armMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getPosition() {
       return armMotor.getSelectedSensorPosition();
    }

    public void setCurrentLimit(StatorCurrentLimitConfiguration currentLimit) {
        armMotor.configStatorCurrentLimit(currentLimit);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        //if (Math.abs(output) < 0.05) {
          //  output = Math.copySign(0.05, output);
        //}
        armMotor.set(ControlMode.PercentOutput, output);
        SmartDashboard.putNumber("Waka Waka Arm Output", output);
    }

    @Override
    protected double getMeasurement() {
        return armMotor.getSelectedSensorPosition();
    }
    
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Waka Waka Arm Ticks", armMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Waka Waka Arm Setpoint", getSetpoint());
        SmartDashboard.putBoolean("Arm Limit Switch", getArmLimit());
    }
}
