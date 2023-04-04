package frc.robot.subsystems.wakaWaka;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.nerdyfiles.utilities.CTREUtils;



public class wakaWakaArm extends PIDSubsystem {
    private TalonFX armMotor = new TalonFX(60);
    private static double kP = 0.00015;
    private static double kI = 0;
    private static double kD = 0;
    private double peakOutput = 1.0;


    public wakaWakaArm() {
        super(new PIDController(kP, kI, kD));
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

        enable();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (Math.abs(output) < 0.05) {
            output = Math.copySign(0.05, output);
        }
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
    }
}
