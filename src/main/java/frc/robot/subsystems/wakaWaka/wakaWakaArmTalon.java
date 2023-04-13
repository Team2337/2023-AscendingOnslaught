package frc.robot.subsystems.wakaWaka;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.nerdyfiles.utilities.CTREUtils;



public class wakaWakaArmTalon extends SubsystemBase {
    private TalonFX armMotor = new TalonFX(60);
    private static double kP = 0.0003;
    private static double kI = 0;
    private static double kD = 0;
    private double peakOutput = 1.0;
    private double setpoint;


    public wakaWakaArmTalon() {
        armMotor.configFactoryDefault();
        armMotor.configForwardSoftLimitThreshold(1600);
        armMotor.configForwardSoftLimitEnable(true);
        armMotor.configReverseSoftLimitThreshold(0);
        armMotor.configReverseSoftLimitEnable(true);
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.configStatorCurrentLimit(CTREUtils.lowCurrentLimit());
        armMotor.configPeakOutputForward(peakOutput, 10);
        armMotor.configPeakOutputReverse(-peakOutput, 10);
        armMotor.setInverted(TalonFXInvertType.Clockwise);
        armMotor.setSelectedSensorPosition(0);
    }
    
    // public void setSetpoint(double setpoint) {
    //     this.setpoint = setpoint;
    // }

    public void setSpeed(double speed) {
        armMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    
    @Override
    public void periodic() {
        // armMotor.set(ControlMode.Position, setpoint);
        SmartDashboard.putNumber("Waka Waka Arm Ticks", armMotor.getSelectedSensorPosition());
    }
}
