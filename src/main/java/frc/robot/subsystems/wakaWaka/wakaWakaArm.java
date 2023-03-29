package frc.robot.subsystems.wakaWaka;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.nerdyfiles.utilities.CTREUtils;



public class wakaWakaArm extends PIDSubsystem {
    private TalonFX armMotor = new TalonFX(67);
    private static double kP = 0.1;
    private static double kI = 0;
    private static double kD = 0;
    private double peakOutput;


    public wakaWakaArm() {
        super(new PIDController(kP, kI, kD));
        armMotor.configFactoryDefault();
        armMotor.configForwardSoftLimitThreshold(0);
        armMotor.configForwardSoftLimitEnable(false);
        armMotor.configReverseSoftLimitThreshold(150000);
        armMotor.configReverseSoftLimitEnable(false);
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit());
        armMotor.configPeakOutputForward(peakOutput, 10);
        armMotor.configPeakOutputReverse(-peakOutput, 10);
        armMotor.setInverted(TalonFXInvertType.Clockwise);
        armMotor.setSelectedSensorPosition(0);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        armMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    protected double getMeasurement() {
        return armMotor.getSelectedSensorPosition();
    }
    
    @Override
    public void periodic() {
        super.periodic();
    }
}
