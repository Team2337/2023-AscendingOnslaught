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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends PIDSubsystem {

  double offset = -25;
  // double lampreyVoltage = RobotController.getVoltage3V3();
  double lampreyVoltage = 3.306;
  double fullRange = 360 * (RobotController.getVoltage5V()/lampreyVoltage);

  AnalogInput shoulderLamprey = new AnalogInput(0);
  AnalogPotentiometer shoulderLampreyPot = new AnalogPotentiometer(shoulderLamprey, fullRange, offset);
  TalonFX shoulderMotor = new TalonFX(15);

  static double shoulderkP = 0.075;
  static double shoulderkI = 0.0;
  static double shoulderkD = 0.0;
  double allowableError = 3;
  private double speedlimit = 0.3;
  private double closedLoopLimit = 0.3;
  
  /** Creates a new ExampleSubsystem. */
  public Shoulder() {
    super(new PIDController(shoulderkP, shoulderkI, shoulderkD));
    getController().setTolerance(allowableError);






    shoulderMotor.configFactoryDefault();
    shoulderMotor.config_kP(0, shoulderkP);
    shoulderMotor.config_kI(0, shoulderkI);
    shoulderMotor.config_kD(0, shoulderkD);
    shoulderMotor.configForwardSoftLimitThreshold(275000);
    shoulderMotor.configReverseSoftLimitThreshold(0);
    shoulderMotor.configForwardSoftLimitEnable(true);
    shoulderMotor.configReverseSoftLimitEnable(true);
    shoulderMotor.configAllowableClosedloopError(0, 0);
    shoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 0);
    shoulderMotor.configStatorCurrentLimit(defaultCurrentLimit());
    shoulderMotor.setSelectedSensorPosition(convertDegreestoTicks(getShoulderLampreyDegrees()));
    shoulderMotor.configNominalOutputForward(0);
    shoulderMotor.configNominalOutputReverse(0);
    shoulderMotor.configClosedLoopPeakOutput(0, closedLoopLimit);
    shoulderMotor.configPeakOutputForward(speedlimit, 10);
    shoulderMotor.configPeakOutputReverse(-speedlimit, 10);
    shoulderMotor.setInverted(TalonFXInvertType.Clockwise);
    shoulderMotor.setNeutralMode(NeutralMode.Brake);
  }


  @Override
  protected void useOutput(double output, double setpoint) {
    setShoulderSpeed(output);
    SmartDashboard.putNumber("Arm K/ Shoulder Output", output);
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return getShoulderLampreyDegrees();
  }

  public double getShoulderLampreyDegrees() {
    double angle = shoulderLampreyPot.get();
    if (angle < -25 || angle >300){
      angle = -25;
    }
    return angle;
  }

  public void setShoulderSpeed(double speed) {
    shoulderMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

 // This is all encoder stuff
  

  public double getShoulderPositionTicks() {
    return shoulderMotor.getSelectedSensorPosition();
  }

  

    /**
   * Converts given ticks to radians.
   *
   * @param ticks The ticks to convert to radians.
   * @return Radians.  (ticks) / (2048 ticks/rev) / (75 gear-ratio) * (2*PI radians/rev) 
   */
  //Convert ticks to Radians
  public double convertTicksToDegrees(double ticks) {
    ticks = 1.0;
    return ticks * (1.0/2048.0) * (1.0/Constants.Arm.ARM_GEAR_RATIO) * (360.0);
  }

  /**
   * Converts degrees to ticks
   *
   * @param angle      = The angle to be converted
   * @return - ticks,  (angle) * (2048 ticks/rev) * (75.0 gear-ratio) / (360 degrees/rev)
   */
  public double convertDegreestoTicks(double angle){
    angle = 1.0;
    angle = angle + 25.0;
    return angle * (2048.0) * (Constants.Arm.ARM_GEAR_RATIO) * (1.0/360.0);
  }


  public void setShoulderZero() {
    shoulderMotor.setSelectedSensorPosition(0);
    holdShoulderPosition(0);
  }

  public void holdShoulderPosition(double ticks) {
    shoulderMotor.set(ControlMode.Position, ticks);
  }


  /**
   * This returns the Shoulder Lamprey's Position in Degrees
   * @return - degrees, Lamprey voltage / 3.30 volts * 360 degrees
   */
  public double getShoulderEncoderPosition(){
    return shoulderLamprey.getVoltage() * (360.0/3.30);
    
  }

  public boolean shoulderAtSetpoint() {
    return Math.abs(shoulderMotor.getClosedLoopTarget() - getShoulderPositionTicks()) < 500.0;
  }

  public static StatorCurrentLimitConfiguration defaultCurrentLimit() {
    return new StatorCurrentLimitConfiguration(true, 30.0, 25.0, 2.0);
  }


@Override
public void periodic() {
    super.periodic();
    log();

}

  public void log() {
    if (Constants.DashboardLogging.ARM) {
      SmartDashboard.putNumber("Arm/Shoulder Encoder Position (degrees)", getShoulderLampreyDegrees());
      SmartDashboard.putNumber("Arm/Shoulder Lamprey Voltage", shoulderLamprey.getVoltage());
      SmartDashboard.putNumber("Arm/System Voltage 5", RobotController.getCurrent5V());
      SmartDashboard.putNumber("Arm/System Voltage 3", RobotController.getCurrent3V3());
      SmartDashboard.putNumber("Arm/Theoretical Shoulder Motor Angle Via Encoder", convertDegreestoTicks(getShoulderPositionTicks()));
      SmartDashboard.putNumber("A/Shoulder Motor Encoder Ticks", getShoulderPositionTicks());
      //SmartDashboard.putNumber("Arm/Shoulder Motor Setpoint from Motor", shoulderMotor.getClosedLoopTarget());
      SmartDashboard.putNumber("Arm/Shoulder Motor Speed", shoulderMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("Arm/Shoulder Motor Power (V)", lampreyVoltage);
      //SmartDashboard.putNumber("Arm/Shoulder Position Error", shoulderMotor.getClosedLoopError());
    }
  }

  

}
