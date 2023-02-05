package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private CANSparkMax intakeMotor = new CANSparkMax(17, MotorType.kBrushless);
    private double peakOutput = 0.4;
    

    public Intake() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
        intakeMotor.setOpenLoopRampRate(peakOutput);
        intakeMotor.setSmartCurrentLimit(40, 40, 40); // TODO CHANGE LATER
    }

    public Command RunIntake() {
        return new StartEndCommand(()->setIntakeSpeed(peakOutput), ()->setIntakeSpeed(0), this);
        
    }

    public Command RunOuttake() {
        return new StartEndCommand(()->setIntakeSpeed(-peakOutput), ()->setIntakeSpeed(0), this);

    }
    public void setIntakeSpeed(double speed) {       
        intakeMotor.set(speed);
    }

    public double getIntakeMotorTemperature() {
        return intakeMotor.getMotorTemperature();
    }
@Override
public void periodic() {
    SmartDashboard.putNumber("motorTemp", getIntakeMotorTemperature());
}
}
