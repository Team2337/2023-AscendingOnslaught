package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
    private double intakeSpeed = 0.5;

    public Intake() {
        intakeMotor.configFactoryDefault();
        intakeMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit());
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.setInverted(TalonFXInvertType.Clockwise);
        intakeMotor.configOpenloopRamp(.5);
        

    }
    public void forward() {
        intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    }

    public void reverse() {
        intakeMotor.set(ControlMode.PercentOutput, -intakeSpeed);
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
    @Override
    public void periodic() {

    }
    
}
