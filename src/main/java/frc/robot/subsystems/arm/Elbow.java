// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Elbow extends PIDSubsystem {

  double offset = -188;
  // double lampreyVoltage = RobotController.getVoltage3V3();
  double lampreyVoltage = 3.306;
  double fullRange = 360 * (RobotController.getVoltage5V()/lampreyVoltage);

  AnalogInput elbowLamprey = new AnalogInput(1);
  AnalogPotentiometer elbowLampreyPot = new AnalogPotentiometer(elbowLamprey, fullRange, offset);
  TalonFX elbowMotor = new TalonFX(16);

  static double elbowkP = 0.025;
  static double elbowkI = 0.0;
  static double elbowkD = 0.0;
  double allowableError = 3;
  private double speedlimit = 0.5;
  private double closedLoopLimit = 0.5;
  
  /** Creates a new ExampleSubsystem. */
  public Elbow() {
    super(new PIDController(elbowkP, elbowkI, elbowkD));
    getController().setTolerance(allowableError);

    elbowMotor.configFactoryDefault();
    elbowMotor.config_kP(0, elbowkP);
    elbowMotor.config_kI(0, elbowkI);
    elbowMotor.config_kD(0, elbowkD);
    elbowMotor.configForwardSoftLimitThreshold(100000);
    elbowMotor.configReverseSoftLimitThreshold(0);
    elbowMotor.configForwardSoftLimitEnable(false);
    elbowMotor.configReverseSoftLimitEnable(false);
    elbowMotor.configAllowableClosedloopError(0, 0);
    elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 0);
    elbowMotor.configStatorCurrentLimit(defaultCurrentLimit());
    elbowMotor.setSelectedSensorPosition(convertDegreestoTicks(getElbowLampreyDegrees()));
    elbowMotor.configNominalOutputForward(0);
    elbowMotor.configNominalOutputReverse(0);
    elbowMotor.configClosedLoopPeakOutput(0, closedLoopLimit);
    elbowMotor.configPeakOutputForward(speedlimit, 10);
    elbowMotor.configPeakOutputReverse(-speedlimit, 10);
    elbowMotor.setInverted(TalonFXInvertType.Clockwise);
    elbowMotor.setNeutralMode(NeutralMode.Brake);
  }


  @Override
  protected void useOutput(double output, double setpoint) {
    setElbowSpeed(output);
    SmartDashboard.putNumber("Arm K/ Elbow Output", output);
  }

  @Override
  protected double getMeasurement() {
    return getElbowLampreyDegrees();
  }

  public double getElbowLampreyDegrees() {
    return elbowLampreyPot.get();
  }

  public void setElbowSpeed(double speed) {
    elbowMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

 // This is all encoder stuff
  

  public double getElbowPositionTicks() {
    return elbowMotor.getSelectedSensorPosition();
  }

  

     /**
   * Converts given ticks to radians.
   *
   * @param ticks The ticks to convert to radians.
   * @return Radians.  (ticks) / (2048 ticks/rev) / (75 gear-ratio) * (2*PI radians/rev) 
   */
  //Convert ticks to Radians
  public double convertTicksToDegrees(double ticks) {
    return ticks * (1.0/2048.0) * (1.0/Constants.Arm.ARM_GEAR_RATIO) * (360.0);
  }

  /**
   * Converts degrees to ticks
   *
   * @param angle      = The angle to be converted
   * @return - ticks,  (angle) * (2048 ticks/rev) * (75.0 gear-ratio) / (360 degrees/rev)
   */
  public double convertDegreestoTicks(double angle){
    angle = angle;
    return angle * (2048.0) * (Constants.Arm.ARM_GEAR_RATIO) * (1.0/360.0);
  }


  public void setElbowZero() {
    elbowMotor.setSelectedSensorPosition(0);
    holdElbowPosition(0);
  }

  public void holdElbowPosition(double ticks) {
    elbowMotor.set(ControlMode.Position, ticks);
  }





  public static StatorCurrentLimitConfiguration defaultCurrentLimit() {
    return new StatorCurrentLimitConfiguration(true, 50.0, 40.0, 2.0);
  }

  @Override
  public void periodic() {
      super.periodic();
      log();
  }


  public void log() {
    if (Constants.DashboardLogging.ARM) {
      SmartDashboard.putNumber("Arm/Elbow Encoder Position (degrees)", getElbowLampreyDegrees());
      SmartDashboard.putNumber("Arm/Elbow Lamprey Voltage", elbowLamprey.getVoltage());
      //SmartDashboard.putNumber("Arm/Theoretical Elbow Motor Angle Via Encoder", convertTicksToRadians(getElbowPositionTicks()));
      SmartDashboard.putNumber("Arm/Elbow Motor Encoder Ticks", getElbowPositionTicks());
      //SmartDashboard.putNumber("Arm/Elbow Motor Setpoint from Motor", elbowMotor.getClosedLoopTarget());
      SmartDashboard.putNumber("Arm/Elbow Motor Speed", elbowMotor.getMotorOutputPercent());
      //SmartDashboard.putNumber("Arm/Elbow Motor Power (V)", elbowMotor.getStatorCurrent());
      //SmartDashboard.putNumber("Arm/Elbow Position Error", elbowMotor.getClosedLoopError());
    }
  }

  

}
