// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AllianceColor;
import frc.robot.Constants.DriverDashboardPositions;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.LEDState;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.commands.CartesianHeadingToTargetCommand;
import frc.robot.commands.LED.LEDRunnable;
import frc.robot.commands.arm.ArmBasicJoystickCommand;
import frc.robot.commands.arm.ArmSetpointCommand;
import frc.robot.commands.arm.ArmSetpointElbow;
import frc.robot.commands.arm.ArmSetpointShoulder;
import frc.robot.commands.arm.ArmSetpointWithEnding;
import frc.robot.commands.arm.ArmSetpointWithIntake;
import frc.robot.commands.arm.intake.IntakeCommand;
import frc.robot.commands.arm.intake.IntakeHoldPosition;
import frc.robot.commands.arm.intake.IntakeUnjam;
import frc.robot.commands.arm.intake.OuttakeCommand;
import frc.robot.commands.arm.intakeSpinner.IntakeSpinnerAdjustment;
import frc.robot.commands.auto.*;
import frc.robot.commands.auto.common.DoNothingCommand;
import frc.robot.commands.auto.pathplanner.blueStartLeftyLeftScoreW1GToppyScoreC2GTopScoreO2Balance;
import frc.robot.commands.auto.pathplanner.blueStartLeftyLeftScoreW1GToppyScoreC2GTopScoreO2SendIt;
import frc.robot.commands.auto.pathplanner.redLeftyLeftScoreW1GToppyScoreC2GTopScoreO2;
import frc.robot.commands.auto.pathplanner.redLeftyLeftScoreW1GToppyScoreC2GTopScoreO2SendIt;
import frc.robot.commands.auto.pathplanner.redRightyRightScoreW9GBotScoreC8GMidScoreO8SendIt;
import frc.robot.commands.auto.pathplanner.redRightyRightScoreW9GBotScoreC8GMidScoreO8Balance;
import frc.robot.commands.auto.teleop.BlueConstructTeleopAutoCommand1;
import frc.robot.commands.auto.teleop.BlueConstructTeleopAutoCommand2;
import frc.robot.commands.auto.teleop.BlueConstructTeleopAutoCommand3;
import frc.robot.commands.auto.teleop.BlueTeleopAutoLeftSubstation;
import frc.robot.commands.auto.teleop.BlueTeleopAutoRightSubstation;
import frc.robot.commands.auto.teleop.RedConstructTeleopAutoCommand1;
import frc.robot.commands.auto.teleop.RedConstructTeleopAutoCommand2;
import frc.robot.commands.auto.teleop.RedConstructTeleopAutoCommand3;
import frc.robot.commands.auto.teleop.RedTeleopAutoLeftSubstation;
import frc.robot.commands.auto.teleop.RedTeleopAutoRightSubstation;
import frc.robot.commands.auto.test.AvoidChargeStation;
import frc.robot.commands.auto.test.Test;
import frc.robot.commands.auto.test.Test3M;
import frc.robot.commands.arm.intakeSpinner.IntakeSpinnerAdjustment;
import frc.robot.commands.swerve.Lockdown;
import frc.robot.commands.swerve.MaintainHeadingCommand;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.nerdyfiles.leds.LED;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class RobotContainer {
  private  GamePiece gamePiece = GamePiece.Nothing;
  private LEDState ledState = LEDState.Nothing;
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);
  public JoystickButton driverRightBumper;
  public JoystickButton operatorLeftBumper;
  public JoystickButton yellowSwitch;

  private final PigeonIMU pigeon = new PigeonIMU(0);

  private final AutoDrive autoDrive = new AutoDrive();
  private final Drivetrain drivetrain = new Drivetrain(pigeon, (GamePiece x) -> setGamePiece(x));
  private final Intake intake = new Intake();
  private final IntakeSpinnerLamprey intakespinner = new IntakeSpinnerLamprey(intake::getIntakeSpinnerLampreyVoltage, this::getGamepiece);
  private final Elbow elbow = new Elbow(this);
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation, drivetrain::isMoving);
  private final LED led = new LED();
  // private final PowerDistributionHub powerDistributionHub = new PowerDistributionHub();
  private final RobotType robotType = new RobotType();
  private final Shoulder shoulder = new Shoulder();
  private final Vision vision = new Vision(this);
 

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final SendableChooser<String> startingPosChooser = new SendableChooser<>();
  private final SendableChooser<Double> startingAngleChooser = new SendableChooser<>();

  private enum CommandSelector {
    ONE,
    TWO,
    THREE
  }

  // public PathPlannerTrajectory Test3MPath;
  // public PathPlannerTrajectory blueLeftyLeftGToppyTop;
  public PathPlannerTrajectory AvoidChargeStation;
  public PathPlannerTrajectory blueLeftyLeftGToppy;
  public PathPlannerTrajectory blueScoreC2;
  public PathPlannerTrajectory blueScoreO2;
  public PathPlannerTrajectory blueChargeStation2;
  public PathPlannerTrajectory blueLockdown;
  public PathPlannerTrajectory blueSendIt;

  public PathPlannerTrajectory redRightyRightGBottom;
  public PathPlannerTrajectory redScoreC8;
  public PathPlannerTrajectory redAvoidChargeStation;
  public PathPlannerTrajectory redScoreO8;
  public PathPlannerTrajectory redChargeStation;
  public PathPlannerTrajectory redLockdown;
  public PathPlannerTrajectory redSendIt;

  public PathPlannerTrajectory redLeftyLeftGToppy;
  public PathPlannerTrajectory redScoreC2;
  public PathPlannerTrajectory redGTop;
  public PathPlannerTrajectory redScoreO2;

  // public PathPlannerTrajectory redLeftyLeftGToppyFast;
  // public PathPlannerTrajectory redScoreC2Fast;
  // public PathPlannerTrajectory redGTopFast;
  // public PathPlannerTrajectory redScoreO2Fast;
  // public PathPlannerTrajectory redLeftSendIt;

  public RobotContainer() {
    operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    yellowSwitch = new JoystickButton(operatorStation, 4);


    // Test3MPath = PathPlanner.loadPath("Test3M", new PathConstraints(4, 3));
    AvoidChargeStation = PathPlanner.loadPath("Avoid Charge Station", new PathConstraints(2.75, 2.5));
    // blueLeftyLeftGToppyTop = PathPlanner.loadPath("blueLeftyLeftGToppyTop", new PathConstraints(3.0, 4.0));
    blueLeftyLeftGToppy = PathPlanner.loadPath("blueLeftyLeftGToppy", new PathConstraints(3.0, 4.0));
    blueScoreC2 = PathPlanner.loadPath("blueScoreC2", new PathConstraints(3.0, 4.0));
    blueScoreO2 = PathPlanner.loadPath("blueScoreO2", new PathConstraints(3.0, 4.0));
    blueChargeStation2 = PathPlanner.loadPath("blueChargeStation2", new PathConstraints(3.0, 2.0));
    blueLockdown = PathPlanner.loadPath("blueLockdown", new PathConstraints(2.0, 1.5));
    blueSendIt = PathPlanner.loadPath("blueSendIt", new PathConstraints(3.0, 4.0));

    redRightyRightGBottom = PathPlanner.loadPath("redRightyRightGBottom", new PathConstraints(3.0, 4.0));
    redScoreC8 = PathPlanner.loadPath("redScoreC8", new PathConstraints(3.0, 3.0));
    redAvoidChargeStation = PathPlanner.loadPath("redAvoidChargeStation", new PathConstraints(3, 3.0));//2.25,4
    redScoreO8 = PathPlanner.loadPath("redScoreO8", new PathConstraints(3.0, 4.0));
    redChargeStation = PathPlanner.loadPath("redChargeStation", new PathConstraints(2.0, 1.5));
    redLockdown = PathPlanner.loadPath("redLockdown", new PathConstraints(2.0, 1.5));
    redSendIt = PathPlanner.loadPath("redSendIt", new PathConstraints(3.0, 4.0));

    redLeftyLeftGToppy = PathPlanner.loadPath("redLeftyLeftGToppy", new PathConstraints(2.0, 2.0));
    redScoreC2 = PathPlanner.loadPath("redScoreC2", new PathConstraints(2.0, 2.0));
    redGTop = PathPlanner.loadPath("redGTop", new PathConstraints(2.0, 2.0));
    redScoreO2 = PathPlanner.loadPath("redScoreO2", new PathConstraints(2.0, 2.0));

    // redLeftyLeftGToppyFast = PathPlanner.loadPath("redLeftyLeftGToppy", new PathConstraints(3.0, 2.0));
    // redScoreC2Fast = PathPlanner.loadPath("redScoreC2", new PathConstraints(3.0, 2.0));
    // redGTopFast = PathPlanner.loadPath("redGTop", new PathConstraints(3.0, 2.0));
    // redScoreO2Fast = PathPlanner.loadPath("redScoreO2", new PathConstraints(3.0, 2.0));
    // redLeftSendIt = PathPlanner.loadPath("redLeftSendIt", new PathConstraints(3.0, 4.0));

    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));
    // heading.setDefaultCommand(
    //     new CartesianHeadingToTargetCommand(drivetrain::getTranslation, operatorLeftBumper::getAsBoolean,
    //         driverRightBumper::getAsBoolean, drivetrain, heading, vision));
    // shoulder.setDefaultCommand(new ArmJoystickCommand(elbow, shoulder, operatorController, ()-> true)); //TODO: This is the override switch for the lamprey failing, please dont let that happen
    shoulder.setDefaultCommand(new ArmBasicJoystickCommand(elbow, shoulder, ()-> operatorController));
    intake.setDefaultCommand(new IntakeHoldPosition(intake));
    // vision.setDefaultCommand(new PeriodicRelocalizeCartesian(drivetrain, vision));
    // elbow.setDefaultCommand(new ArmBasicJoystickCommand(elbow, shoulder, () ->
    // operatorController));
    led.setDefaultCommand(new LEDRunnable(led, this, ()->intake.hasCone()).ignoringDisable(true));
    // Configure the button bindings
    configureButtonBindings();

    // Create auton selector
    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());

    // autonChooser.addOption("Arm Test", new armTest(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));

    // autonChooser.addOption("Red Middle Right Score 1 Grab 1 Balance", new redStartMiddleRightScoreO6Balance(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Red Righty Right Score 1 Grab 1 Balance", new redStartRightyRightScoreO9GBotBalance(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Red Righty Right Score 2 Balance", new redStartRightyRightScoreO9GBotScoreO7Balance(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Red Righty Right Score 2 Grab 1", new redStartRightyRightScoreO9GBotScoreO8GMid(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));

    // autonChooser.addOption("Blue Middle Left Score 1 Grab 1 Balance", new blueStartMiddleLeftScoreO4GTopBalance(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Blue Lefty Left Score 1 Grab 1 Balance", new blueStartLeftyLeftScoreO1GToppyBalance(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Blue Lefty Left Score 2 Balance", new blueStartLeftyLeftScoreO1GToppyScoreO3Balance(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Blue Lefty Left Score 2 Grab 1", new blueStartLeftyLeftScoreO1GToppyScoreO2GTop(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Blue Lefty Left Score 3 Balance Yoshi", new blueStartLeftyLeftScoreW1GToppyScoreO2GTopScoreC2BalanceYoshi(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Blue Lefty Left Score 2 Grab 1 Balance Yoshi", new blueStartLeftyLeftScoreO1GToppyScoreO2GTopBalanceYoshi(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Blue Lefty Left Score 3 Yoshi", new blueStartLeftyLeftScoreO1GToppyScoreO2GTopScoreC2Yoshi(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Blue Lefty Left Score 3 Cube Yoshi", new blueStartLeftyLeftScoreW1GToppyScoreO2GTopScoreC2Yoshi(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));

    
    // // autonChooser.addOption("Test3M", new Test3M(Test3MPath, autoDrive, drivetrain, heading));
    // autonChooser.addOption("Test", new Test(autoDrive, drivetrain, heading, elbow, intake, intakespinner, shoulder, this));
    // autonChooser.addOption("Avoid Charge Station", new AvoidChargeStation(AvoidChargeStation, autoDrive, drivetrain, heading));
    
    autonChooser.addOption("Red Righty Right Score 3 Balance", new redRightyRightScoreW9GBotScoreC8GMidScoreO8Balance(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    autonChooser.addOption("Red Righty Right Score 3 Yeet", new redRightyRightScoreW9GBotScoreC8GMidScoreO8SendIt(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    autonChooser.addOption("Red Lefty Left Score 3", new redLeftyLeftScoreW1GToppyScoreC2GTopScoreO2(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    // autonChooser.addOption("Red Lefty Left Score 3 Yeet", new redLeftyLeftScoreW1GToppyScoreC2GTopScoreO2SendIt(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    
    autonChooser.addOption("Blue Lefty Left Score 3 Balance", new blueStartLeftyLeftScoreW1GToppyScoreC2GTopScoreO2Balance(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));
    autonChooser.addOption("Blue Lefty Left Score 3 Yeet", new blueStartLeftyLeftScoreW1GToppyScoreC2GTopScoreO2SendIt(autoDrive, drivetrain, elbow, heading, intake, intakespinner, this, shoulder));

    SmartDashboard.putData("AutonChooser", autonChooser);

    startingPosChooser.addOption("Zero", "Zero");
    startingPosChooser.addOption("0.5,0.5", "0.5,0.5");
    startingPosChooser.addOption("Right Right", "Right Right");
    startingPosChooser.setDefaultOption("Right Middle", "Right Middle");
    startingPosChooser.addOption("Right Left", "Right Left");
    startingPosChooser.addOption("Middle Right", "Middle Right");
    startingPosChooser.addOption("Middle Middle", "Middle Middle");
    startingPosChooser.addOption("Middle Left", "Middle Left");
    startingPosChooser.addOption("Left Right", "Left Right");
    startingPosChooser.addOption("Left Middle", "Left Middle");
    startingPosChooser.addOption("Left Left", "Left Left");


    SmartDashboard.putData("StartingPositionChooser", startingPosChooser);

    startingAngleChooser.setDefaultOption("180 degrees", 180.0);
    startingAngleChooser.addOption("0 degrees", 0.0);

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
    Constants.DRIVER_DASHBOARD.addBoolean("Alliance", () -> AllianceColor.getAllianceColor() == AllianceColor.Red)
        .withPosition(DriverDashboardPositions.ALLIANCE.x, DriverDashboardPositions.ALLIANCE.y)
        .withSize(3, 3)
        .withProperties(Map.of("Color when true", "#ff3333", "Color when false", "#3333ff"));

    Constants.DRIVER_DASHBOARD.addNumber("Shoulder Lamprey", () -> shoulder.getShoulderLampreyDegrees())
      .withPosition(DriverDashboardPositions.SHOULDERLAMPREY.x, DriverDashboardPositions.SHOULDERLAMPREY.y)
      .withSize(3, 3);

    Constants.DRIVER_DASHBOARD.addNumber("Elbow Lamprey", () -> elbow.getElbowLampreyDegrees())
      .withPosition(DriverDashboardPositions.ELBOWLAMPREY.x, DriverDashboardPositions.ELBOWLAMPREY.y)
      .withSize(3, 3);   
      
    Constants.DRIVER_DASHBOARD.addNumber("Intake Spinner Lamprey", () -> intakespinner.getEncoderDegrees())
      .withPosition(DriverDashboardPositions.INTAKESPINNERLAMPREY.x, DriverDashboardPositions.INTAKESPINNERLAMPREY.y)
      .withSize(3, 3);

    Constants.DRIVER_DASHBOARD.addBoolean("Intake Spinner Sensor", () -> intake.hasCone())
      .withPosition(DriverDashboardPositions.INTAKESENSOR.x, DriverDashboardPositions.INTAKESENSOR.y)
      .withSize(3, 3);

    Constants.DRIVER_DASHBOARD.addString("Past Arm Position", () -> shoulder.pastPosition.toString())
      .withPosition(DriverDashboardPositions.PASTARMPOSITION.x, DriverDashboardPositions.PASTARMPOSITION.y)
      .withSize(6, 3);
  }

  public void resetRobot2023() {
    // Other option here is Constants.STARTING_ANGLE for booting against Hub
    pigeon.setYaw(0, 250);
    drivetrain.resetPosition(
        new Pose2d(
            Constants.Auto.zeroPoint,
            new Rotation2d(0)));
  }

  public void resetRobotTeleop() {
    drivetrain.resetPosition(
        new Pose2d(
            Constants.Auto.zeroPoint,
            new Rotation2d(0)));
  }

  private CommandSelector selectTeleopAuto() {
    if (driverController.getYButton()) {
      return CommandSelector.ONE;
    } else if (driverController.getXButton()) {
      return CommandSelector.TWO;
    } else {
      return CommandSelector.THREE;
    }
  }

  public void resetRobotChooser(String startPos, double startingAngle) {
    if (startPos == "0.5,0.5") {
      drivetrain.resetPosition(new Pose2d(new Translation2d(0.5, 0.5), Rotation2d.fromDegrees(startingAngle)));
      return;
    }
    if (DriverStation.getAlliance() == Alliance.Blue) {
      switch (startPos) {

        case "Zero":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.zeroPoint, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Right":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.blueGridRightRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Middle":
          drivetrain.resetPosition(
              new Pose2d(Constants.Auto.blueGridRightRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Left":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.blueGridRightRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Right":
          drivetrain.resetPosition(
              new Pose2d(Constants.Auto.blueGridMiddleRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Middle":
          drivetrain.resetPosition(
              new Pose2d(Constants.Auto.blueGridMiddleRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Left":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.blueGridMiddleRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Right":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.blueGridLeftRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Middle":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.blueGridLeftRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Left":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.blueGridLeftRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        default:
          drivetrain.resetPosition(new Pose2d(Constants.Auto.zeroPoint, new Rotation2d(startingAngle)));
          break;
      }
    } else if (DriverStation.getAlliance() == Alliance.Red) {
      switch (startPos) {

        case "Zero":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.zeroPoint, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Right":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.redGridRightRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Middle":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.redGridRightRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Left":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.redGridRightRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Right":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.redGridMiddleRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Middle":
          drivetrain.resetPosition(
              new Pose2d(Constants.Auto.redGridMiddleRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Left":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.redGridMiddleRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Right":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.redGridLeftRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Middle":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.redGridLeftRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Left":
          drivetrain
              .resetPosition(new Pose2d(Constants.Auto.redGridLeftRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        default:
          drivetrain.resetPosition(new Pose2d(Constants.Auto.zeroPoint, new Rotation2d(startingAngle)));
          break;
      }
    }
  }

  public void enableMaintainHeading() {
    heading.enableMaintainHeading();
  }

  private void configureButtonBindings() {
    //SmartDashboard.putData(CommandScheduler.getInstance());
    /** Driver Controller */
    // Note: Left X + Y axis, Right X axis, and Left Bumper are used by
    // SwerveDriveCommand to turn on/off field orientation

    JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
    JoystickButton driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
    JoystickButton driverB = new JoystickButton(driverController, XboxController.Button.kB.value);
    JoystickButton driverY = new JoystickButton(driverController, XboxController.Button.kY.value);
    JoystickButton driverBack = new JoystickButton(driverController, XboxController.Button.kBack.value);
    JoystickButton driverStart = new JoystickButton(driverController, XboxController.Button.kStart.value);
    Trigger triggerDriverRight = new Trigger(() -> driverController.getRightTriggerAxis() > 0.5);
    Trigger triggerDriverLeft = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5);
    driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    JoystickButton driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    
    driverRightBumper.onTrue(new MaintainHeadingCommand(0, heading));
    //If blue 90, if red -90
    driverLeftBumper.onTrue(new ConditionalCommand(new MaintainHeadingCommand(90, heading), 
    new MaintainHeadingCommand(-90, heading), 
    () -> Constants.AllianceColor.getAllianceColor() == AllianceColor.Blue));

    triggerDriverRight.whileTrue(new OuttakeCommand(intake, this));
    triggerDriverLeft.whileTrue(new IntakeUnjam(intake));

    // driverA.whileTrue(new SelectCommand(
    //       // Maps selector values to commands
    //       Map.ofEntries(
    //           Map.entry(CommandSelector.ONE,  new ConditionalCommand(new BlueConstructTeleopAutoCommand1(autoDrive, drivetrain, heading),
    //           new RedConstructTeleopAutoCommand1(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue)),
    //           Map.entry(CommandSelector.TWO,  new ConditionalCommand(new BlueConstructTeleopAutoCommand2(autoDrive, drivetrain, heading),
    //           new RedConstructTeleopAutoCommand2(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue)),
    //           Map.entry(CommandSelector.THREE,  new ConditionalCommand(new BlueConstructTeleopAutoCommand3(autoDrive, drivetrain, heading),
    //           new RedConstructTeleopAutoCommand3(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue))),
    //       this::selectTeleopAuto));

    driverB.whileTrue(new Lockdown(autoDrive, drivetrain, heading).withTimeout(0.2));

    // driverLeftBumper.whileTrue(new ConditionalCommand(new BlueTeleopAutoLeftSubstation(autoDrive, drivetrain, heading),
    //   new  RedTeleopAutoLeftSubstation(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue));
    // driverRightBumper.whileTrue(new ConditionalCommand(new  BlueTeleopAutoRightSubstation(autoDrive, drivetrain, heading),
    //   new  RedTeleopAutoRightSubstation(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue));
    
    /** Operator Controller * */
    // Note: Left X axis is used by DeliveryOverrideCommand

    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);
    JoystickButton operatorRightStick = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);
    JoystickButton operatorLeftStick = new JoystickButton(operatorController, XboxController.Button.kLeftStick.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    Trigger triggerOperatorRight = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5);
    Trigger triggerOperatorLeft = new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5);
    Trigger operatorPOVUp = new Trigger(() -> operatorController.getPOV() == 0);
    Trigger operatorPOVDown = new Trigger(() -> operatorController.getPOV() == 180);
    Trigger operatorPOVRight = new Trigger(() -> operatorController.getPOV() == 90);
    JoystickButton operatorBack = new JoystickButton(operatorController, XboxController.Button.kBack.value);
    JoystickButton operatorStart = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    

    triggerOperatorRight.whileTrue(new IntakeCommand(this::getGamepiece, (LEDState x) -> setLEDState(x), intake));
    triggerOperatorRight.whileTrue(new ConditionalCommand( 
      new ArmSetpointWithIntake(Constants.Arm.ArmPosition.SUBSTATIONPICKUP, this::getGamepiece, elbow, shoulder, intakespinner), 
      new WaitCommand(0), 
      () -> wasPastPositionSubstation()));
    
    triggerOperatorLeft.whileTrue(new OuttakeCommand(intake, this));
    operatorRightBumper.whileTrue(new ArmSetpointShoulder(Constants.Arm.ArmPosition.SUBSTATION, 95, elbow, shoulder, intakespinner, this).andThen(new ArmSetpointCommand(Constants.Arm.ArmPosition.SUBSTATION, elbow, shoulder, intakespinner, this)));
    operatorLeftBumper.whileTrue(new ConditionalCommand(
      new ArmSetpointElbow(Constants.Arm.ArmPosition.ALTERNATEINTERMEDIATE, 5, elbow, shoulder, intakespinner, this).andThen(new ArmSetpointShoulder(Constants.Arm.ArmPosition.ALTERNATECARRY, elbow, shoulder, intakespinner, this)).andThen(new ArmSetpointCommand(Constants.Arm.ArmPosition.ALTERNATECARRYEND, elbow, shoulder, intakespinner, this)),
        new ConditionalCommand(
        new ArmSetpointElbow(Constants.Arm.ArmPosition.SUBSTATIONCARRY, 230, elbow, shoulder, intakespinner, this).andThen(new ArmSetpointCommand(Constants.Arm.ArmPosition.SUBSTATIONCARRY, elbow, shoulder, intakespinner, this)),
        new ArmSetpointWithEnding(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, 45, elbow, shoulder, intakespinner, this).andThen(new ArmSetpointCommand(Constants.Arm.ArmPosition.CARRY, elbow, shoulder, intakespinner, this)),
        ()-> wasPastPositionSubstation()),
      ()-> wasPastPositionFloorPickup())
      );

    operatorY.whileTrue(new ArmSetpointShoulder(Constants.Arm.ArmPosition.SCOREHIGH, elbow, shoulder, intakespinner, this).andThen(new ArmSetpointCommand(Constants.Arm.ArmPosition.SCOREHIGH, elbow, shoulder, intakespinner, this)));
    operatorB.whileTrue(new ArmSetpointShoulder(Constants.Arm.ArmPosition.SCOREMID, elbow, shoulder, intakespinner, this).andThen(new ArmSetpointCommand(Constants.Arm.ArmPosition.SCOREMID, elbow, shoulder, intakespinner, this)));
    operatorA.whileTrue(new ArmSetpointShoulder(Constants.Arm.ArmPosition.SCORELOW, elbow, shoulder, intakespinner, this).andThen(new ArmSetpointCommand(Constants.Arm.ArmPosition.SCORELOW, elbow, shoulder, intakespinner, this)));
    operatorX.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.FEEDSTATIONFRONT, elbow, shoulder, intakespinner, this));
    operatorPOVRight.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.SCORESIDEPICKUPLOW, elbow, shoulder, intakespinner, this));
 
    operatorStart.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.TELEFALLINGCONE, elbow, shoulder, intakespinner, this));

    operatorBack.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.TELESTANDINGCONE, elbow, shoulder, intakespinner, this));

    operatorPOVUp.onTrue(new IntakeSpinnerAdjustment(intakespinner, Constants.Arm.WRIST_ANGLE_ADJUSTMENT));
    operatorPOVDown.onTrue(new IntakeSpinnerAdjustment(intakespinner, -Constants.Arm.WRIST_ANGLE_ADJUSTMENT));
    
    operatorRightStick.onTrue(new InstantCommand(()-> setGamePiece(GamePiece.Cone)).andThen(new InstantCommand(() -> setLEDState(LEDState.Cone))));
    operatorLeftStick.onTrue(new InstantCommand(()-> setGamePiece(GamePiece.Cube)).andThen(new InstantCommand(() -> setLEDState(LEDState.Cube))));

    // operatorX.whileTrue(new ArmJoystickCommand(elbow, shoulder, operatorController, ()->getYellowSwitchStatus()));

    // TODO: Create switch to flip between orange and blue
    JoystickButton yellowButton = new JoystickButton(operatorStation, 10);
    JoystickButton purpleButton = new JoystickButton(operatorStation, 11);

  }

  public void instantiateSubsystemsTeleop() {
    heading.setDefaultCommand(
         new CartesianHeadingToTargetCommand(drivetrain::getTranslation, yellowSwitch::getAsBoolean,
             driverRightBumper::getAsBoolean, drivetrain, heading, vision));
  }
  public void setArmPeakOutput(double peak) {
    elbow.changePeakOutput(peak);
    shoulder.changePeakOutput(peak);
  }

  public void setGamePiece(GamePiece gamePiece){
    this.gamePiece = gamePiece;
  }
  public GamePiece getGamepiece() {
    return gamePiece;
  }
  public LEDState getLEDState() {
    return ledState;
  }
  public void setLEDState(LEDState ledState) {
    this.ledState = ledState;
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

  public boolean getBlackSwitchStatus() {
    return operatorStation.blackSwitch.getAsBoolean();
  }
  /**
   * This is the switch for the arm lamprey's failing
   * @return
   */
  public boolean getYellowSwitchStatus() {
    return operatorStation.yellowSwitch.getAsBoolean();
  }

  public boolean getClearSwitchStatus() {
    return operatorStation.clearSwitch.getAsBoolean();
  }

  public boolean getDriverInput() {
    return (Math.abs(driverController.getRightX()) > 0.2) || (Math.abs(driverController.getLeftY()) > 0.2);
  }


  public int getElbowSetpoint() {
    return (int) elbow.getSetpoint();
  }

  public int getShoulderSetpoint() {
    return (int) shoulder.getSetpoint();
  }

  public double getOpYRight() {
    return operatorController.getRightY();
  }

  public double getOpYLeft() {
    return operatorController.getLeftY();
  }

  public boolean wasPastPositionSubstation() {
    return (shoulder.pastPosition == ArmPosition.SUBSTATION || shoulder.pastPosition == ArmPosition.SUBSTATIONPICKUP);
  }

  public boolean wasPastPositionFloorPickup() {
    return (shoulder.pastPosition == ArmPosition.TELEFALLINGCONE || shoulder.pastPosition == ArmPosition.TELESTANDINGCONE);
  }

  public double getShoulderAngle() {
    return shoulder.getShoulderLampreyDegrees();
  }

  public ArmPosition setPastArmPositionFallenCone() {
    return shoulder.pastPosition = ArmPosition.TELEFALLINGCONE;
  }
}
