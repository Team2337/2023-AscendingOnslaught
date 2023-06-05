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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elbow extends PIDSubsystem {

  double offset = -188;
  // double lampreyVoltage = RobotController.getVoltage3V3();
  double lampreyVoltage = 3.306;
  double fullRange = 360 * (RobotController.getVoltage5V()/lampreyVoltage);

  AnalogInput elbowLamprey = new AnalogInput(1);
  AnalogPotentiometer elbowLampreyPot = new AnalogPotentiometer(elbowLamprey, fullRange, offset);
  TalonFX elbowMotor = new TalonFX(16);

  private RobotContainer robotContainer;

  static double elbowkP = 0.018;
  static double elbowkI = 0.0;
  static double elbowkD = 0.0;
  double allowableError = 0.2;
  private double speedlimit = Constants.Arm.ELBOW_MAX_SPEED;
  private double closedLoopLimit = Constants.Arm.ELBOW_CLOSED_LOOP_SPEED;
  
  /** Creates a new ExampleSubsystem. */
  public Elbow(RobotContainer robotContainer) {
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

    this.robotContainer = robotContainer;
  }


  @Override
  protected void useOutput(double output, double setpoint) {
    if (DriverStation.isTeleop()) {
      double ff = 0.05 * Math.cos(Units.degreesToRadians(getElbowLampreyDegrees() + robotContainer.getShoulderAngle()));
      setElbowSpeed(output + ff);
      SmartDashboard.putNumber("Arm/ Elbow Output", output + ff);
    } else {
      setElbowSpeed(output);
    }
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

  public void changePeakOutput(double peak) {
    elbowMotor.configPeakOutputForward(peak, 10);
    elbowMotor.configPeakOutputReverse(-peak, 10);
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

  public void setElbowSetpoint(double setpoint, double p) {
    if (DriverStation.isTeleop()) {
      getController().setP(p);
    }
    setSetpoint(setpoint);
  }

  @Override
  public void periodic() {
      super.periodic();
      log();
      // getController().setP(0.025 + 0.03 * Math.abs(Math.cos(Units.degreesToRadians(getElbowLampreyDegrees() + robotContainer.getShoulderAngle()))));
  }


  public void log() {
    if (Constants.DashboardLogging.ELBOW) {
      SmartDashboard.putNumber("Arm/Elbow Lamprey Voltage", elbowLamprey.getVoltage());
      //SmartDashboard.putNumber("Arm/Theoretical Elbow Motor Angle Via Encoder", convertTicksToRadians(getElbowPositionTicks()));
      SmartDashboard.putNumber("Arm/Elbow Motor Encoder Ticks", getElbowPositionTicks());
      //SmartDashboard.putNumber("Arm/Elbow Motor Setpoint from Motor", elbowMotor.getClosedLoopTarget());
      SmartDashboard.putNumber("Arm/Elbow Motor Speed", elbowMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("Arm/Elbow Setpoint", m_controller.getSetpoint());
      //SmartDashboard.putNumber("Arm/Elbow Motor Power (V)", elbowMotor.getStatorCurrent());
      // SmartDashboard.putNumber("Arm K/ Elbow Output", output);
      //SmartDashboard.putNumber("Arm/Elbow Position Error", elbowMotor.getClosedLoopError());
      SmartDashboard.putNumber("Elbow Setpoint and Actual Angle Difference", m_controller.getSetpoint() - getElbowLampreyDegrees());
      SmartDashboard.putNumber("Elbow Velocity Error", m_controller.getVelocityError());
      SmartDashboard.putNumber("Arm/Elbow P", m_controller.getP());
    }
    SmartDashboard.putNumber("Arm/Elbow Encoder Degrees", getElbowLampreyDegrees());
  }

  

}
