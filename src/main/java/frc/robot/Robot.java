// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.arm.Shoulder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private String startingPos = "UnSet";
  private double startingAngle = 110.0;

  private RobotContainer m_robotContainer;
  private boolean autonomousRan = false;
  public static String allianceColor = "hello";


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_robotContainer.resetRobot2023();

    
  }

  /*
   * private void setupLogger() {
   * Logger logger = Logger.getInstance();
   * 
   * // Run as fast as possible during replay
   * setUseTiming(isReal());
   * // Log & replay "SmartDashboard" values (no tables are logged by default).
   * LoggedNetworkTables.getInstance().addTable("/SmartDashboard");
   * // Set a metadata value
   * logger.recordMetadata("ProjectName", "2022Relentless");
   * 
   * // Log to USB stick (name will be selected automatically)
   * // logger.addDataReceiver(new ByteLogReceiver("/media/sda1/"));
   * // Provide log data over the network, viewable in Advantage Scope.
   * logger.addDataReceiver(new LogSocketServer(5800));
   * 
   * // Start logging! No more data receivers, replay sources, or metadata values
   * may
   * // be added.
   * logger.start();
   * }
   */



  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // SmartDashboard.putBoolean("auto", autonomousRan);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //m_topMotor.set(0.0);
    //m_bottomMotor.set(0.0);
  }

  @Override
  public void disabledPeriodic() {
    startingPos = m_robotContainer.getStartingPosition();
    SmartDashboard.putString("Starting Position", startingPos);
    startingAngle = m_robotContainer.getStartingAngle();
    SmartDashboard.putNumber("Starting Angle", startingAngle);
    SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().toString());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.resetRobotChooser(startingPos, startingAngle);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // TODO: do we need to add a pause between setting pose and starting auton?

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    autonomousRan = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.enableMaintainHeading();

    if (!autonomousRan) {
      m_robotContainer.resetRobotTeleop();
    }
    // TODO: Used for april tag testing, do not remove
    // m_robotContainer.resetRobotChooser(startingPos, startingAngle);

    // m_robotContainer.instantiateSubsystemsTeleop();
    // m_robotContainer.configureButtonBindingsTeleop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Alliance Color", allianceColor);
    allianceColor = DriverStation.getAlliance().toString();
/* 
    switch (controlMode.getSelected()) {

      case 1:
        // Here, we run PID control where the top arm acts like a four-bar relative to
        // the bottom. Maybe?
        topSetpoint = (int) (MathUtil.clamp(SmartDashboard.getNumber("Setpoint top (degrees)", 0) - MathUtil.clamp(
            SmartDashboard.getNumber("Setpoint bottom (degrees)", 150), m_arm_bottom_min_angle, m_arm_bottom_max_angle),
            m_arm_top_min_angle, m_arm_top_max_angle));
        bottomSetpoint = (int) MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0),
            m_arm_bottom_min_angle, m_arm_bottom_max_angle);
        break;
      case 2:
        // Set setpoints manually in SmartDashboard
        topSetpoint = (int) MathUtil.clamp(SmartDashboard.getNumber("Setpoint top (degrees)", 0), m_arm_top_min_angle,
            m_arm_top_max_angle);
        bottomSetpoint = (int) MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0),
            m_arm_bottom_min_angle, m_arm_bottom_max_angle);
        break;
      case 3:
        /**
         * // calculate X and/or Y based on current angles - adjust X and/or Y based on
         * joystick - calaculate new angles (topSetpoint,bottomSetpoint),
         * deltaX = m_joystick.getRawAxis(0) * .1;
         * deltaY = m_joystick.getRawAxis(1) * .1;
         * SmartDashboard.putNumber("deltaX", deltaX);
         * double currentX = m_arm_bottomLength *
         * Math.cos(m_bottomEncoder.getDistance()) + m_arm_topLength *
         * Math.cos(m_bottomEncoder.getDistance() + m_topEncoder.getDistance());
         * double currentY = m_arm_bottomLength *
         * Math.sin(m_bottomEncoder.getDistance()) + m_arm_topLength *
         * Math.sin(m_bottomEncoder.getDistance() + m_topEncoder.getDistance());
         * SmartDashboard.putNumber("currentX", currentX);
         * double targetX = currentX + deltaX;
         * double targetY = currentY + deltaY;
         * SmartDashboard.putNumber("targetX", targetX);
         * //calculate angles to get to X,Y
         * double hypot = Math.sqrt((targetX * targetX) + (targetY * targetY));
         * double theta_S2 = Math.acos((m_arm_bottomLength*m_arm_bottomLength +
         * hypot*hypot - m_arm_topLength*m_arm_topLength) /
         * (2*hypot*m_arm_bottomLength));
         * double theta_S1 = Math.atan2( targetY, targetX);
         * bottomSetpoint = (int) Units.radiansToDegrees(theta_S1 + theta_S2);
         * double theta_E = Math.acos((m_arm_bottomLength*m_arm_bottomLength +
         * m_arm_topLength*m_arm_topLength - hypot*hypot) /
         * (2*m_arm_bottomLength*m_arm_topLength));
         * topSetpoint = (int) Units.radiansToDegrees(theta_E - 180);
         * double theta_F = Math.acos((hypot*hypot + m_arm_topLength*m_arm_topLength -
         * m_arm_bottomLength*m_arm_bottomLength) / (2*hypot*m_arm_topLength));
         * SmartDashboard.putNumber("A+B+C", Units.radiansToDegrees(theta_F + theta_E +
         * theta_S1));
         * SmartDashboard.putNumber("sin(90)", Math.sin(90));
         * SmartDashboard.putNumber("sin(pi 2) correct", Math.sin(Math.PI/2));
         **/
        // =====================

        // Convert sensor readings to angles as used in our forward and inverse
        // kinematics.
        // Shoulder angle shoud be zero when level with the ground and pointing straight
        // back from the robot (when back of the robot to the right, angles are positive
        // CCW)
       // Convert sensor readings to angles as used in our forward and inverse kinematics.
        // Shoulder angle shoud be zero when level with the ground and pointing straight back from the robot (when back of the robot to the right, angles are positive CCW)
       /*  double mathShoulderAngle = Units.radiansToDegrees(m_bottomEncoderSim.getDistance());
        double joystickP = 3;
        // Elbow Angle is zero when parallel with first/bottom arm (when back of the robot to the right, angles are positive CCW)
        double mathElbowAngle = Units.radiansToDegrees(m_topEncoderSim.getDistance());
        // Calculate the current X,Y location of the intake
        double currentX = Constants.Arm.SHOULDER_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle)) + Constants.Arm.ELBOW_ARM_LENGTH * Math.cos(Units.degreesToRadians(mathShoulderAngle) + Units.degreesToRadians(mathElbowAngle));
        double currentY = Constants.Arm.SHOULDER_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathShoulderAngle)) + Constants.Arm.ELBOW_ARM_LENGTH * Math.sin(Units.degreesToRadians(mathShoulderAngle) + Units.degreesToRadians(mathElbowAngle));

        
        //Read Joystick inputs and apply a deadband
        // TODO:  do we want X and Y on different joysticks??  variable for deadband?
        double joystickX = Utilities.deadbandAndSquare(-m_robotContainer.getOpYRight(), 0.1);
        double joystickY = Utilities.deadbandAndSquare(-m_robotContainer.getOpYLeft(), 0.1);

        deltaX = joystickX * joystickP; 
        deltaY = joystickY * joystickP;
        //deltaX = 0;
        //deltaY = 0;

        // Adjust the target X,Y location of the intake based on joystick inputs
        double targetX = currentX + deltaX;
        double targetY = currentY + deltaY;
        //targetX = -33;
        //targetY = 25;
        SmartDashboard.putNumber("Arm K/DeltaX", deltaX);
        SmartDashboard.putNumber("Arm K/DeltaY", deltaY);
        SmartDashboard.putNumber("Arm K/Target X", targetX);
        SmartDashboard.putNumber("Arm K/Target Y", targetY);
        
        // Calculate new arm angles based on target X,Y    
        double hypot = Math.sqrt((targetX * targetX) + (targetY * targetY));
        double theta_S2 = Math.acos((Math.pow(Constants.Arm.SHOULDER_ARM_LENGTH, 2) + Math.pow(hypot, 2) - Math.pow(Constants.Arm.ELBOW_ARM_LENGTH,2)) 
        / (2.0 * hypot * Constants.Arm.SHOULDER_ARM_LENGTH));
       // double theta_S1 = Math.asin(targetY/hypot);
       double theta_S1 = Math.atan2(targetY, targetX);
        double theta_E = Math.acos((Math.pow(Constants.Arm.SHOULDER_ARM_LENGTH, 2) + Math.pow(Constants.Arm.ELBOW_ARM_LENGTH, 2) - Math.pow(hypot,2)) 
        / (2.0 * Constants.Arm.SHOULDER_ARM_LENGTH * Constants.Arm.ELBOW_ARM_LENGTH));
        SmartDashboard.putNumber("Arm K/hypot", hypot);
        SmartDashboard.putNumber("Arm K/theta_S2", Units.radiansToDegrees(theta_S2));
        SmartDashboard.putNumber("Arm K/theta_S1", Units.radiansToDegrees(theta_S1));
        SmartDashboard.putNumber("Arm K/theta_E", Units.radiansToDegrees(theta_E));

        // TODO: NOT SURE ON LOGIC, HAVE TO WALK THRU AGAIN
        // Final steps to determine new angle setpoints differs based on the quadrant (x,y) is in.
       /*  if (targetY >= 0) {
            bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1 + theta_S2));
        }
        else {
            bottomSetpoint = (int)(Units.radiansToDegrees(theta_S2 + theta_S1));
        } */
        /* 
        if (targetX < 0) {
            topSetpoint = (int)(Units.radiansToDegrees(Math.PI - theta_E));
            bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1-theta_S2));
            }
        else {
                topSetpoint = (int)(Units.radiansToDegrees(theta_E-Math.PI));
                bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1 + theta_S2));
        }
        if ((joystickX == 0) && (joystickY == 0)){ 
          if (shouldHoldArm) {
              topSetpoint = (int)(Units.radiansToDegrees(m_topEncoderSim.getDistance()));
              bottomSetpoint = (int)(Units.radiansToDegrees(m_bottomEncoderSim.getDistance()));
              shouldHoldArm = false;
          }
      } else {
          // Change boolean so that if joysticks go back to zero, we will get position/set setpoint, stoping the arms movement.
          shouldHoldArm = true;    
      }   
                /*else {
            topSetpoint = (int)(Units.radiansToDegrees(theta_E - Math.PI));
        } */

        //TODO: THINK WE NEED TO SUBTRACT OUT OFFSETS BEFORE SETTING POSITIONS
       // double shoulderTarget = bottomSetpoint;
        //double elbowTarget = topSetpoint;


        // ===============
/* 
        break;

      case 4:
        // Set setpoints manually in SmartDashboard
        topSetpoint = m_robotContainer.getElbowSetpoint();
        bottomSetpoint = m_robotContainer.getShoulderSetpoint();
        break;
      default: // also case 0, use predefined setpoints

        switch (presetChooser.getSelected()) {
          case 0:
            topSetpoint = startingPositionTop;
            bottomSetpoint = startingPositionBottom;
            break;
          case 1:
            topSetpoint = intakeTop;
            bottomSetpoint = intakeBottom;
            break;
          case 2:
            topSetpoint = doubleSubstationTop;
            bottomSetpoint = doubleSubstationBottom;
            break;
          case 3:
            topSetpoint = scoreFloorTop;
            bottomSetpoint = scoreFloorBottom;
            break;
          case 4:
            topSetpoint = scoreMidTop;
            bottomSetpoint = scoreMidBottom;
            break;
          case 5:
            topSetpoint = scoreHighTop;
            bottomSetpoint = scoreHighBottom;
            break;
          case 6:
            topSetpoint = straightUpTop;
            bottomSetpoint = straightUpBottom;
            break;
          case 7:
            topSetpoint = scoreTravelTop;
            bottomSetpoint = scoreTravelBottom;
            break;
          case 8:
            topSetpoint = intakeTravelTop;
            bottomSetpoint = intakeTravelBottom;
            break;
          case 9:
            topSetpoint = -30;
            bottomSetpoint = doubleSubstationBottom;
            break;
          default:
            topSetpoint = scoreTravelTop;
            bottomSetpoint = scoreTravelBottom;
            break;
        }
        break;
    }
    // Here, we run PID control where the arm moves to the selected setpoint.
    pidOutputTop = m_topController.calculate(m_topEncoder.getDistance(), Units.degreesToRadians(topSetpoint));
    m_topMotor.setVoltage(pidOutputTop);

    pidOutputBottom = m_bottomController.calculate(m_bottomEncoder.getDistance(),
        Units.degreesToRadians(bottomSetpoint));
    m_bottomMotor.setVoltage(pidOutputBottom);

    // VARIOUS SMARTDASHBOARD PRINTS
    SmartDashboard.putNumber("top length", ELBOW_ARM_LENGTH);
    SmartDashboard.putNumber("bottom length", SHOULDER_ARM_LENGTH);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", bottomSetpoint);
    SmartDashboard.putNumber("Setpoint top (degrees)", topSetpoint);
    SmartDashboard.putNumber("get bottom encoder", Units.radiansToDegrees(m_bottomEncoder.getDistance()));
    SmartDashboard.putNumber("get top encoder", Units.radiansToDegrees(m_topEncoder.getDistance()));
    SmartDashboard.putNumber("CurrentX", SHOULDER_ARM_LENGTH * Math.sin(m_bottomEncoder.getDistance())
        + ELBOW_ARM_LENGTH * Math.sin(m_bottomEncoder.getDistance() + m_topEncoder.getDistance()));
    SmartDashboard.putNumber("CurrentY", SHOULDER_ARM_LENGTH * Math.cos(m_bottomEncoder.getDistance())
        + ELBOW_ARM_LENGTH * Math.cos(m_bottomEncoder.getDistance() + m_topEncoder.getDistance())); */

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
