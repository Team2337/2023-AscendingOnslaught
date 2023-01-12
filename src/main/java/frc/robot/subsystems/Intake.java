package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private final TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);



    public Intake(){

    }

    @Override
    public void periodic(){

    }
    
}
