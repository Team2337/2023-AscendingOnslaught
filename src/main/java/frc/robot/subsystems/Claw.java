package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Claw extends SubsystemBase {

    private final TalonFX clawMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
    private double clawSpeed = 0.5;

    public Claw() {
        clawMotor.configFactoryDefault();
        clawMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit());
        clawMotor.setNeutralMode(NeutralMode.Coast);
        clawMotor.setInverted(TalonFXInvertType.Clockwise);
        clawMotor.configOpenloopRamp(.5);
        

    }
    public void open() {
        clawMotor.set(ControlMode.PercentOutput, clawSpeed);
    }

    public void close() {
        clawMotor.set(ControlMode.PercentOutput, -clawSpeed);
    }

    public void stop() {
        clawMotor.set(ControlMode.PercentOutput, 0);
    }
    @Override
    public void periodic() {

    }
    
}
