package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private CANSparkMax intakeMotor = new CANSparkMax(17, MotorType.kBrushless);
    private double peakOutput = 0.8;
    

    public Intake() {

        intakeMotor.setInverted(false);
        intakeMotor.setOpenLoopRampRate(peakOutput);
        intakeMotor.setSmartCurrentLimit(40, 40, 40); // TODO CHANGE LATER

    }

    public void setIntakeSpeed(double speed) {       
        intakeMotor.set(speed);
    }

    public double getintakeMotorTemperature() {
        return intakeMotor.getMotorTemperature();
    }

}
