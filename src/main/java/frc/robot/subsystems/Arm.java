// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  AnalogInput shoulderLamprey = new AnalogInput(0);
  AnalogInput elbowLamprey = new AnalogInput(1);
  TalonFX shoulderMotor = new TalonFX(16);
  TalonFX elbowMotor = new TalonFX(3);

  double elbowkP = 0.5;
  double elbowkI = 0.0;
  double elbowkD = 0.0;

  double shoulderkP = 0.5;
  double shoulderkI = 0.0;
  double shoulderkD = 0.0;

  double shoulderOffset = 0;
  double elbowOffset = 0;

  
  /** Creates a new ExampleSubsystem. */
  public Arm() {
    shoulderMotor.configFactoryDefault();
    shoulderMotor.config_kP(0, shoulderkP);
    shoulderMotor.config_kI(0, shoulderkI);
    shoulderMotor.config_kD(0, shoulderkD);
    shoulderMotor.configForwardSoftLimitThreshold(100000);
    shoulderMotor.configReverseSoftLimitThreshold(0);
    shoulderMotor.configForwardSoftLimitEnable(true);
    shoulderMotor.configReverseSoftLimitEnable(true);
    shoulderMotor.configAllowableClosedloopError(0, 0);
    shoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 0);
    shoulderMotor.configStatorCurrentLimit(defaultCurrentLimit());
    shoulderMotor.setSelectedSensorPosition(0);
    shoulderMotor.configNominalOutputForward(0);
    shoulderMotor.configNominalOutputReverse(0);
    shoulderMotor.configClosedLoopPeakOutput(0, Constants.Arm.CLOSED_LOOP_PEAK_OUTPUT);
    shoulderMotor.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT, 10);
    shoulderMotor.configPeakOutputReverse(-Constants.Arm.PEAK_OUTPUT, 10);
    shoulderMotor.setInverted(TalonFXInvertType.Clockwise);
    shoulderMotor.setNeutralMode(NeutralMode.Brake);

    elbowMotor.configFactoryDefault();
    elbowMotor.config_kP(0, elbowkP);
    elbowMotor.config_kI(0, elbowkI);
    elbowMotor.config_kD(0, elbowkD);
    elbowMotor.configAllowableClosedloopError(0, 0);
    elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 10);
    elbowMotor.configStatorCurrentLimit(defaultCurrentLimit());
    elbowMotor.setSelectedSensorPosition(0);
    elbowMotor.configNominalOutputForward(0);
    elbowMotor.configNominalOutputReverse(0);
    elbowMotor.configClosedLoopPeakOutput(0, Constants.Arm.CLOSED_LOOP_PEAK_OUTPUT);
    elbowMotor.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT, 10);
    elbowMotor.configPeakOutputReverse(-Constants.Arm.PEAK_OUTPUT, 10);
    elbowMotor.setInverted(TalonFXInvertType.Clockwise);
    elbowMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Methods for Talon
   */

   public static StatorCurrentLimitConfiguration defaultCurrentLimit() {
    return new StatorCurrentLimitConfiguration(true, 50.0, 40.0, 2.0);
  }

  public void setElbowSpeed(double speed) {
    elbowMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setShoulderSpeed(double speed) {
    shoulderMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getShoulderPositionTicks() {
    return shoulderMotor.getSelectedSensorPosition();
  }
  
  public void setElbowZero() {
    elbowMotor.setSelectedSensorPosition(0);
    holdElbowPosition(0, 0);
  }

  public void setShoulderZero() {
    shoulderMotor.setSelectedSensorPosition(0);
    holdShoulderPosition(0);
  }

  public double getElbowPositionTicks() {
    return elbowMotor.getSelectedSensorPosition();
  }

  public void holdElbowPosition(double elbowTicks, double shoulderTicks) {
    double elbowTarget = (elbowTicks - elbowOffset) + shoulderTicks;
    elbowMotor.set(ControlMode.Position, elbowTarget);
  }

  public void holdShoulderPosition(double ticks) {
    double shoulderTarget = ticks - shoulderOffset;
    shoulderMotor.set(ControlMode.Position, shoulderTarget);
  }

  public double getShoulderPosition() {
    double angle = Units.radiansToDegrees(convertTicksToRadians(getShoulderPositionTicks())) + shoulderOffset;
    return angle;
  }

  /*
   * This is getting an angle with Talon FX's with a virtual fourbar
   */
  public double getElbowPosition() {
    double angle = Units.radiansToDegrees(convertTicksToRadians(getElbowPositionTicks())) - getShoulderPosition() + elbowOffset;
    return angle;
  }


  /**
   * This returns the Shoulder Lamprey's Position in Degrees
   * @return - degrees, Lamprey voltage / 3.30 volts * 360 degrees
   */
  public double getShoulderEncoderPosition(){
    return shoulderLamprey.getVoltage() * (360.0/3.30);
    
  }

  public double getElbowEncoderPosition(){
    return shoulderLamprey.getVoltage() * (360.0/3.30);
    
  }

  public boolean shoulderAtSetpoint() {
    return Math.abs(shoulderMotor.getClosedLoopTarget() - getShoulderPositionTicks()) < 500.0;
  }

  public boolean elbowAtSetpoint() {
    return Math.abs(elbowMotor.getClosedLoopTarget()-getElbowPositionTicks()) < 500.0;
  }

  /**
   * Utility Methods
   */

   /**
   * Converts given ticks to radians.
   *
   * @param ticks The ticks to convert to radians.
   * @return Radians.  (ticks) / (2048 ticks/rev) / (75 gear-ratio) * (2*PI radians/rev) 
   */
  //Convert ticks to Radians
  public double convertTicksToRadians(double ticks) {
    return ticks * (1/2048.0) * (1/75.0) * (2.0 * Math.PI);
  }

  /**
   * Converts degrees to ticks
   *
   * @param angle      = The angle to be converted
   * @return - ticks,  (angle) * (2048 ticks/rev) * (75.0 gear-ratio) / (360 degrees/rev)
   */
  public double convertAnglestoTicks(double angle){
    return angle * (2048.0) * (75.0) * (1.0/360.0);
  }




  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  public void log() {
    if (Constants.DashboardLogging.ARM) {
      SmartDashboard.putNumber("Arm/Shoulder Encoder Position (degrees)", getShoulderEncoderPosition());
      SmartDashboard.putNumber("Arm/Elbow Encoder Position (degrees)", getElbowEncoderPosition());
      SmartDashboard.putNumber("Arm/Theoretical Shoulder Motor Angle Via Encoder", convertTicksToRadians(getShoulderPositionTicks()));
      SmartDashboard.putNumber("Arm/Theoretical Elbow Motor Angle Via Encoder", convertTicksToRadians(getShoulderPositionTicks()));
      SmartDashboard.putNumber("A/Shoulder Motor Encoder Ticks", getShoulderPositionTicks());
      SmartDashboard.putNumber("A/Elbow Motor Encoder Ticks", getElbowPositionTicks());
      SmartDashboard.putNumber("Arm/Shoulder Motor Setpoint from Motor", shoulderMotor.getClosedLoopTarget());
      SmartDashboard.putNumber("Arm/Elbow Motor Setpoint from Motor", elbowMotor.getClosedLoopTarget());
      SmartDashboard.putNumber("Arm/Shoulder Motor Speed", shoulderMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("Arm/Elbow Motor Speed", elbowMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("Arm/Shoulder Motor Power (V)", shoulderMotor.getStatorCurrent());
      SmartDashboard.putNumber("Arm/Elbow Motor Power (V)", elbowMotor.getStatorCurrent());
      SmartDashboard.putNumber("Arm/Shoulder Position Error", shoulderMotor.getClosedLoopError());
      SmartDashboard.putNumber("Arm/Elbow Position Error", elbowMotor.getClosedLoopError());
    }
  }

}
