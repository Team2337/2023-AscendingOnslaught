package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Wrist extends SubsystemBase {

    private final TalonFX wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID);

    public Wrist() {
        wristMotor.configFactoryDefault();
        wristMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit());
        wristMotor.setNeutralMode(NeutralMode.Coast);
        wristMotor.setInverted(TalonFXInvertType.Clockwise);
        wristMotor.configOpenloopRamp(.5);
    }

    public void setSpeed(double speed){
        wristMotor.set(ControlMode.PercentOutput, speed);
    }
    public void stop(){
        wristMotor.set(ControlMode.PercentOutput, 0);
    }
}
