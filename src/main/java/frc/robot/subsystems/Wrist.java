package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Wrist extends SubsystemBase {
    
    private TalonFX wristMotor = new TalonFX(15);
    private double peakOutput = 0.8;
    
    public Wrist() {

        wristMotor.configFactoryDefault();
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit());
        wristMotor.configPeakOutputForward(peakOutput, 10);
        wristMotor.configPeakOutputReverse(-peakOutput, 10);
        
        wristMotor.setInverted(TalonFXInvertType.Clockwise);
    }

    public void setIntakeSpeed(double speed) {       
        wristMotor.set(ControlMode.PercentOutput, speed);
    }   
    public double getFarRollerTemperature() {
        return wristMotor.getTemperature();
    }
    
    

}
