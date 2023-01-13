// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.DriverDashboardPositions;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.claw.StopClaw;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.commands.vision.InstantRelocalizeCommand;
import frc.robot.commands.vision.LimelightHeadingAndInstantRelocalizeCommand;
import frc.robot.commands.vision.PeriodicRelocalizeCommand;
import frc.robot.commands.wrist.WristJoystickCommand;
import frc.robot.nerdyfiles.oi.JoystickAnalogButton;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);

  private final PigeonIMU pigeon = new PigeonIMU(0);

  private final AutoDrive autoDrive = new AutoDrive();
  private final Drivetrain drivetrain = new Drivetrain(pigeon);
  private final Vision vision = new Vision();
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation, drivetrain::isMoving);
  private final Intake intake = new Intake();
  private final Claw claw = new Claw();
  private final Wrist wrist = new Wrist();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final SendableChooser<String> startingPosChooser = new SendableChooser<>();
  private final SendableChooser<Double> startingAngleChooser = new SendableChooser<>();

  public RobotContainer() {
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));
    heading.setDefaultCommand(
        new HeadingToTargetCommand(drivetrain::getTranslation, operatorLeftBumper::getAsBoolean, driverRightBumper::getAsBoolean, drivetrain, heading, vision));
    vision.setDefaultCommand(new PeriodicRelocalizeCommand(drivetrain, vision));
    wrist.setDefaultCommand(new WristJoystickCommand(wrist, operatorController));

    // Configure the button bindings
    configureButtonBindings();

    // Create auton selector
    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());
    
    SmartDashboard.putData("AutonChooser", autonChooser);

    startingPosChooser.setDefaultOption("Right Pos3", "Right");
    startingPosChooser.addOption("Left Pos1", "Left");
    startingPosChooser.addOption("Middle Pos2", "Middle");
    startingPosChooser.addOption("Far Right", "Far Right");

    SmartDashboard.putData("StartingPositionChooser", startingPosChooser);

    startingAngleChooser.addOption("Launchpad (0 degrees)", 0.0);
    startingAngleChooser.addOption("Left fender (-20 degrees)", -20.0);
    startingAngleChooser.addOption("Right fender (70 degrees)", 70.0);
    startingAngleChooser.setDefaultOption("Cargo exit (25 degrees)", 25.0);
    startingAngleChooser.addOption("Right Pos3 Errored Start (80 degrees)", 80.0);
    startingAngleChooser.addOption("Left Pos1 Errored Start (-35 degrees)", -35.0);
    startingAngleChooser.addOption("Middle Pos2 Errored Start (45 degrees)", 45.0);
    startingAngleChooser.addOption("Test (-35 degrees)", -35.0);

    SmartDashboard.putData("StartingAngleChooser", startingAngleChooser);

    // Add dropdowns to driver dashboard
    Constants.DRIVER_DASHBOARD.add("Auton Chooser", autonChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(DriverDashboardPositions.AUTON_CHOOSER.x, DriverDashboardPositions.AUTON_CHOOSER.y)
        .withSize(DriverDashboardPositions.AUTON_CHOOSER.width, DriverDashboardPositions.AUTON_CHOOSER.height);

    Constants.DRIVER_DASHBOARD.add("Starting Pos Chooser", startingPosChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(DriverDashboardPositions.STARTING_POS_CHOOSER.x, DriverDashboardPositions.STARTING_POS_CHOOSER.y)
        .withSize(DriverDashboardPositions.STARTING_POS_CHOOSER.width,
            DriverDashboardPositions.STARTING_POS_CHOOSER.height);

    Constants.DRIVER_DASHBOARD.add("Starting Angle Chooser", startingAngleChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(DriverDashboardPositions.STARTING_ANGLE_CHOOSER.x,
            DriverDashboardPositions.STARTING_ANGLE_CHOOSER.y)
        .withSize(DriverDashboardPositions.STARTING_ANGLE_CHOOSER.width,
            DriverDashboardPositions.STARTING_ANGLE_CHOOSER.height);

    // Put alliance on driver dashboard
    Constants.DRIVER_DASHBOARD.addBoolean("Alliance", () -> BallColor.getAllianceColor() == BallColor.Red)
        .withPosition(DriverDashboardPositions.ALLIANCE.x, DriverDashboardPositions.ALLIANCE.y)
        .withSize(3, 3)
        .withProperties(Map.of("Color when true", "#ff3333", "Color when false", "#3333ff"));

    /*
     * if (Constants.DO_SYSTEMS_CHECK) {
     * Constants.SYSTEMS_CHECK_TAB.addBoolean("Pixy Cam Connected",
     * pixyCam::isConnected)
     * .withPosition(SystemsCheckPositions.PIXY_CAM.x,
     * SystemsCheckPositions.PIXY_CAM.y)
     * .withSize(3, 3);
     * }
     */
  }

  public void resetRobot() {
    // Other option here is Constants.STARTING_ANGLE for booting against Hub
    pigeon.setYaw(0, 250);
    drivetrain.resetPosition(
        new Pose2d(
            Constants.Auto.kStartAtZero.toFieldCoordinate(),
            drivetrain.getGyroscopeRotation()));
  }

  public void resetRobotTeleop() {
    pigeon.setYaw(0, 250);
    drivetrain.resetPosition(
        new Pose2d(
            Constants.Auto.kResetToZero.toFieldCoordinate(),
            drivetrain.getGyroscopeRotation()));
  }

  public void resetRobotAuto() {
    pigeon.setYaw(-35, 250);
    drivetrain.resetPosition(
        new Pose2d(Constants.Auto.kPosition1LeftStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
  }

  public void resetRobotAuto(double startingAngle) {
    pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250);
    drivetrain.resetPosition(
        new Pose2d(Constants.Auto.kPosition3RightStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
  }

  public void resetRobotChooser(String startPos, double startingAngle) {
    switch (startPos) {

      case "Left":
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // -32.25 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPosition1LeftStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;

      case "Middle":
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // 45 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPosition2MiddleStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;

      case "Right":
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // 75 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPosition3RightStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;

      case "Far Right":
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // 90 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPositionFarRightStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;

      default:
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // 80 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPosition3RightStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;
    }
  }

  public void enableMaintainHeading() {
    heading.enableMaintainHeading();
  }

  private void configureButtonBindings() {
    /** Driver Controller */
    // Note: Left X + Y axis, Right X axis, and Left Bumper are used by
    // SwerveDriveCommand
    JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
    JoystickButton driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
    JoystickButton driverB = new JoystickButton(driverController, XboxController.Button.kB.value);
    JoystickButton driverY = new JoystickButton(driverController, XboxController.Button.kY.value);
    JoystickAnalogButton driverTriggerLeft = new JoystickAnalogButton(driverController,
        XboxController.Axis.kLeftTrigger.value);
    JoystickAnalogButton driverTriggerRight = new JoystickAnalogButton(driverController,
        XboxController.Axis.kRightTrigger.value);
    JoystickButton driverBack = new JoystickButton(driverController, XboxController.Button.kBack.value);
    JoystickButton driverStart = new JoystickButton(driverController, XboxController.Button.kStart.value);

    driverA.whenPressed(heading::enableMaintainHeading);

    // driverLeftBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.BLUE,
    // delivery, kicker));
    // driverRightBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.RED,
    // delivery, kicker));
    JoystickButton operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);

    driverBack.whenPressed(new InstantRelocalizeCommand(drivetrain, vision));

    /** Operator Controller * */
    // Note: Left X axis is used by DeliveryOverrideCommand

    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorRightStick = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);
    JoystickButton operatorLeftStick = new JoystickButton(operatorController, XboxController.Button.kLeftStick.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);

    // Operator left bumper used for vision tracking by default commands.
    // Operator right bumper below in the configureButtonBindingsTeleop() method.
    JoystickAnalogButton operatorLeftTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kLeftTrigger.value);
    JoystickAnalogButton operatorRightTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kRightTrigger.value);
    JoystickButton operatorBack = new JoystickButton(operatorController, XboxController.Button.kBack.value);
    JoystickButton operatorStart = new JoystickButton(operatorController, XboxController.Button.kStart.value);

    JoystickButton yellowSwitch = new JoystickButton(operatorStation, 4);
    JoystickButton yellowButton = new JoystickButton(operatorStation, 8);
    JoystickButton blueButton = new JoystickButton(operatorStation, 9);

    operatorRightStick.whileHeld(new LimelightHeadingAndInstantRelocalizeCommand(drivetrain, heading, vision));
    operatorA.onTrue(new StartIntake(intake));
    operatorA.onFalse(new StopIntake(intake));
    operatorB.onTrue(new ReverseIntake(intake));
    operatorB.onFalse(new StopIntake(intake));

    operatorX.onTrue(new OpenClaw(claw).withTimeout(1));
    operatorX.onFalse(new CloseClaw(claw).withTimeout(1));
   

    /** Driverstation Controls * */
  }

  public void instantiateSubsystemsTeleop() {
    // pixyCam = new PixyCam();
    System.out.println("Hello World!");
  }

  public void configureButtonBindingsTeleop() {
    JoystickButton redLeftSwitch = new JoystickButton(operatorStation, 11);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public String getStartingPosition() {
    return startingPosChooser.getSelected();
  }

  public double getStartingAngle() {
    return startingAngleChooser.getSelected();
  }

  public double getGyroscopeRotation() {
    return drivetrain.getGyroscopeRotation().getDegrees();
  }

  public double getGyroscopeRoll() {
    return drivetrain.getGyroscopeRoll().getDegrees();
  }

  public PigeonState getPigeonState() {
    return drivetrain.getPigeonState();
  }

  public boolean isOnTarget() {
    return vision.isOnTarget();
  }

  public boolean hasActiveTarget() {
    return vision.hasActiveTarget();
  }

  public double getTx() {
    return vision.getTx();
  }

  public boolean getBlackSwitchStatus() {
    return operatorStation.blackSwitch.getAsBoolean();
  }

  public boolean getYellowSwitchStatus() {
    return operatorStation.yellowSwitch.getAsBoolean();
  }

  public boolean getClearSwitchStatus() {
    return operatorStation.clearSwitch.getAsBoolean();
  }

}
