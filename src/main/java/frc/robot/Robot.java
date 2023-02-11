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

  static double topArmLength = 27; // in
  static double bottomArmLength = 27; // in
  static double armPivotX = 55; // 65 in = center of 30.5 bumper starting at 49.75
  static double armPivotY = 21.75;// in

  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  //private static final int kJoystickPort = 0;

  // The P gain for the PID controller that drives this arm.
  private static final double kArmKp = 40.0;
  private static final double kArmKi = 0.0;

  double pidOutputTop, pidOutputBottom;
  int topSetpoint, bottomSetpoint;
  boolean shouldHoldArm = true;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  // = (2 * PI rads) / (4096 pulses)
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_topController = new ProfiledPIDController(kArmKp, kArmKi, 0,
      new TrapezoidProfile.Constraints(2, 5));
  private final ProfiledPIDController m_bottomController = new ProfiledPIDController(kArmKp, kArmKi, 0,
      new TrapezoidProfile.Constraints(2, 5));
  private final Encoder m_topEncoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final Encoder m_bottomEncoder = new Encoder(kEncoderAChannel + 2, kEncoderBChannel + 2);

  private final PWMSparkMax m_topMotor = new PWMSparkMax(kMotorPort);
  private final PWMSparkMax m_bottomMotor = new PWMSparkMax(kMotorPort + 1);
  //private final XboxController m_joystick = new XboxController(kJoystickPort);
  double deltaX, deltaY;

  // Simulation classes help us simulate what's going on, including gravity.
  private static final double m_armReduction = 600;
  private static final double m_arm_topMass = 10.0; // Kilograms
  private static final double ELBOW_ARM_LENGTH = Units.inchesToMeters(topArmLength);
  private static final double m_arm_bottomMass = 4.0; // Kilograms
  private static final double SHOULDER_ARM_LENGTH = Units.inchesToMeters(bottomArmLength);

  private static final int m_arm_top_min_angle = -175;
  private static final int m_arm_top_max_angle = 175;
  private static final int m_arm_bottom_min_angle = -30;
  private static final int m_arm_bottom_max_angle = 210;

  // SETPOINTS FOR PRESETS MODE (Uses Virtual 4 Bar Mode for smooth movement)
  private static final int startingPositionBottom = -30;
  private static final int startingPositionTop = 175;

  private static final int straightUpBottom = 90;
  private static final int straightUpTop = 0;

  private static final int scoreTravelBottom = -30;
  private static final int scoreTravelTop = 150;// 260;

  private static final int intakeBottom = 35;
  private static final int intakeTop = -110;// 265;

  private static final int intakeTravelBottom = 65;
  private static final int intakeTravelTop = -143;// 265;

  private static final int doubleSubstationBottom = 98;// 120//60//60
  private static final int doubleSubstationTop = -105;// -125//125//185;

  private static final int scoreFloorBottom = 120;
  private static final int scoreFloorTop = 150;// 255;

  private static final int scoreMidBottom = 80;
  private static final int scoreMidTop = 115;// 195;

  private static final int scoreHighBottom = 110;
  private static final int scoreHighTop = 60;// 160;

  // This arm sim represents an arm that can travel from -75 degrees (rotated down
  // front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_arm_topSim = new SingleJointedArmSim(
      m_armGearbox,
      m_armReduction,
      SingleJointedArmSim.estimateMOI(ELBOW_ARM_LENGTH, m_arm_topMass),
      ELBOW_ARM_LENGTH,
      Units.degreesToRadians(m_arm_top_min_angle),
      Units.degreesToRadians(m_arm_top_max_angle),
      m_arm_topMass,
      false,
      VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );
  private final SingleJointedArmSim m_arm_bottomSim = new SingleJointedArmSim(
      m_armGearbox,
      m_armReduction,
      SingleJointedArmSim.estimateMOI(SHOULDER_ARM_LENGTH, m_arm_bottomMass),
      SHOULDER_ARM_LENGTH,
      Units.degreesToRadians(m_arm_bottom_min_angle),
      Units.degreesToRadians(m_arm_bottom_max_angle),
      m_arm_bottomMass,
      true,
      VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );
  private final EncoderSim m_topEncoderSim = new EncoderSim(m_topEncoder);
  private final EncoderSim m_bottomEncoderSim = new EncoderSim(m_bottomEncoder);
  SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
  SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.

  private final Mechanism2d m_mech2d = new Mechanism2d(90, 90);
  private final MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0);
  private final MechanismLigament2d MidNode = midNodeHome
      .append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0);
  private final MechanismLigament2d HighNode = highNodeHome
      .append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0);
  private final MechanismLigament2d GridNode = gridHome
      .append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d dsHome = m_mech2d.getRoot("Double Substation Home", 80.25, 37);// 49.75 + bumper length
                                                                                               // of 30.5
  private final MechanismLigament2d DSRamp = dsHome
      .append(new MechanismLigament2d("Double Substation Ramp", 13.75, 0, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", armPivotX, armPivotY);
  private final MechanismLigament2d m_arm_bottom = m_armPivot.append(
      new MechanismLigament2d(
          "Arm Bottom",
          bottomArmLength,
          -90,
          10,
          new Color8Bit(Color.kHotPink)));
  private final MechanismLigament2d m_arm_tower = m_armPivot
      .append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kSilver)));

  private final MechanismLigament2d m_aframe_1 = m_armPivot
      .append(new MechanismLigament2d("aframe1", 24, -50, 10, new Color8Bit(Color.kSilver)));
  private final MechanismLigament2d m_bumper = gridHome
      .append(new MechanismLigament2d("Bumper", 30.5, 0, 60, new Color8Bit(Color.kRed)));
  private final MechanismLigament2d m_arm_top = m_arm_bottom.append(
      new MechanismLigament2d(
          "Arm Top",
          topArmLength + 3.0,
          Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
          10,
          new Color8Bit(Color.kPurple)));
  private final MechanismLigament2d m_intake = m_arm_top.append(
      new MechanismLigament2d(
          "Intake",
          7,
          Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
          40,
          new Color8Bit(Color.kYellow)));

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

    m_topEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    m_bottomEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);

    SmartDashboard.putNumber("Setpoint top (degrees)", 90);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", 90);

    controlMode.setDefaultOption("Presets (Setpoints)", 0);
    controlMode.addOption("Virtual Four Bar", 1);
    controlMode.addOption("Manual Angle Adjust", 2);
    controlMode.addOption("Joystick X:Y Adjust", 3);
    controlMode.addOption("Code Adjust", 4);

    presetChooser.setDefaultOption("Starting Position", 0);
    presetChooser.addOption("Floor Intake Position", 1);
    presetChooser.addOption("Double Substation Intake", 2);
    presetChooser.addOption("Floor Node Score", 3);
    presetChooser.addOption("Mid Node Score", 4);
    presetChooser.addOption("High Node Score", 5);
    presetChooser.addOption("Straight Up", 6);
    presetChooser.addOption("Score Travel", 7);
    presetChooser.addOption("Intake Travel", 8);
    presetChooser.addOption("Pre-Score Travel", 9);

    SmartDashboard.putData(controlMode);
    SmartDashboard.putData(presetChooser);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
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

  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_arm_topSim.setInput(m_topMotor.get() * RobotController.getBatteryVoltage());
    m_arm_bottomSim.setInput(m_bottomMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_arm_topSim.update(0.020);
    m_arm_bottomSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_topEncoderSim.setDistance(m_arm_topSim.getAngleRads());
    m_bottomEncoderSim.setDistance(m_arm_bottomSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_arm_topSim.getCurrentDrawAmps() + m_arm_bottomSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm_top.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));
    m_arm_bottom.setAngle(Units.radiansToDegrees(m_arm_bottomSim.getAngleRads()));
  }

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
    m_topMotor.set(0.0);
    m_bottomMotor.set(0.0);
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
        /* 
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
         *
        // =====================
      */
        // Convert sensor readings to angles as used in our forward and inverse
        // kinematics.
        // Shoulder angle shoud be zero when level with the ground and pointing straight
        // back from the robot (when back of the robot to the right, angles are positive
        // CCW)
       // Convert sensor readings to angles as used in our forward and inverse kinematics.
        // Shoulder angle shoud be zero when level with the ground and pointing straight back from the robot (when back of the robot to the right, angles are positive CCW)
        double mathShoulderAngle = Units.radiansToDegrees(m_bottomEncoderSim.getDistance());
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
       //  if (targetY >= 0) {
         //   bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1 + theta_S2));
        //}
        //else {
          //  bottomSetpoint = (int)(Units.radiansToDegrees(theta_S2 + theta_S1));
        //} 
        
        if (targetX < 0) {
            topSetpoint = (int)(Units.radiansToDegrees(Math.PI - theta_E));
            if (targetY < 0){
                bottomSetpoint = (int)(Units.radiansToDegrees((2*Math.PI) + theta_S1-theta_S2));
            }
            else {
                bottomSetpoint = (int)(Units.radiansToDegrees(theta_S1-theta_S2));
            }
            
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
        + ELBOW_ARM_LENGTH * Math.cos(m_bottomEncoder.getDistance() + m_topEncoder.getDistance())); 

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
