// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AllianceColor;
import frc.robot.Constants.DriverDashboardPositions;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.CartesianHeadingToTargetCommand;
import frc.robot.commands.LED.LEDRunnable;
import frc.robot.commands.arm.ArmBasicJoystickCommand;
import frc.robot.commands.arm.ArmJoystickCommand;
import frc.robot.commands.arm.ArmSetpointCommand;
import frc.robot.commands.arm.ArmSetpointWithEnding;
import frc.robot.commands.arm.intake.IntakeCommand;
import frc.robot.commands.arm.intake.OuttakeCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.auto.common.DoNothingCommand;
import frc.robot.commands.auto.drive.CartesianProfiledPointToPointCommand;
import frc.robot.commands.auto.drive.CartesianVectorProfileToPointCommand;
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
import frc.robot.commands.auto.test.AngleTest;
import frc.robot.commands.auto.test.MoveForwardTest;
import frc.robot.commands.auto.test.Test;
import frc.robot.commands.intakeSpinner.IntakeSpinnerAdjustment;
import frc.robot.commands.intakeSpinner.IntakeSpinnerSetPoint;
import frc.robot.commands.auto.test.vectorBlueRightMiddleToBottom;
import frc.robot.commands.swerve.MaintainHeadingCommand;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.nerdyfiles.leds.LED;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.commands.vision.InstantRelocalizeCartesianCommand;
import frc.robot.commands.vision.InstantRelocalizeCommand;
import frc.robot.commands.vision.LimelightHeadingAndInstantRelocalizeCommand;
import frc.robot.commands.vision.PeriodicRelocalizeCartesian;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.Elbow;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shoulder;

public class RobotContainer {
  private  GamePiece gamePiece = GamePiece.Nothing;
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);

  private final PigeonIMU pigeon = new PigeonIMU(0);

  private final AutoDrive autoDrive = new AutoDrive();
  private final Drivetrain drivetrain = new Drivetrain(pigeon, (GamePiece x) -> setGamePiece(x));
  private final Intake intake = new Intake();
  private final IntakeSpinnerLamprey intakespinner = new IntakeSpinnerLamprey(intake::getIntakeSpinnerLampreyVoltage, this::getGamepiece);
  private final Elbow elbow = new Elbow();
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

  public RobotContainer() {
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));
    heading.setDefaultCommand(
        new CartesianHeadingToTargetCommand(drivetrain::getTranslation, operatorLeftBumper::getAsBoolean,
            driverRightBumper::getAsBoolean, drivetrain, heading, vision));
                   // shoulder.setDefaultCommand(new ArmJoystickCommand(elbow, shoulder, operatorController, ()-> true)); //TODO: This is the override switch for the lamprey failing, please dont let that happen
    shoulder.setDefaultCommand(new ArmBasicJoystickCommand(elbow, shoulder, ()-> operatorController));
    vision.setDefaultCommand(new PeriodicRelocalizeCartesian(drivetrain, vision));
    // elbow.setDefaultCommand(new ArmBasicJoystickCommand(elbow, shoulder, () ->
    // operatorController));
    led.setDefaultCommand(new LEDRunnable(led, this, ()->intake.hasCone()).ignoringDisable(true));
    // Configure the button bindings
    configureButtonBindings();

    // Create auton selector
    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());
    // autonChooser.addOption("Test", new Test(autoDrive, drivetrain, heading));
    // autonChooser.addOption("Angle Test", new AngleTest(autoDrive, drivetrain, heading));
    // autonChooser.addOption("Move Test", new blueRightMiddleToBottom(autoDrive, drivetrain, heading));
    // autonChooser.addOption("Move Forward Test", new MoveForwardTest(autoDrive, drivetrain, heading));
    // autonChooser.addOption("Vector Test", new vectorBlueRightMiddleToBottom(autoDrive, drivetrain, heading));
    // autonChooser.addOption("Full Field Straight Vector", new CartesianVectorProfileToPointCommand(
    //     new Translation2d(16, 0), drivetrain::getTranslation, 1.5, Units.inchesToMeters(80), autoDrive, heading));
    // autonChooser.addOption("Full Field Straight XY",
    //     new CartesianProfiledPointToPointCommand(new Translation2d(16, 0), drivetrain::getTranslation,
    //         drivetrain::getRotation, 1.5, 1.5, Units.inchesToMeters(80), Units.inchesToMeters(80), autoDrive, heading));
    // autonChooser.addOption("Full Field Diagonal Vector", new CartesianVectorProfileToPointCommand(
    //     new Translation2d(16, 8), drivetrain::getTranslation, 1.5, Units.inchesToMeters(80), autoDrive, heading));
    // autonChooser.addOption("Full Field Diagonal XY",
    //     new CartesianProfiledPointToPointCommand(new Translation2d(16, 8), drivetrain::getTranslation,
    //         drivetrain::getRotation, 1.5, 1.5, Units.inchesToMeters(80), Units.inchesToMeters(80), autoDrive, heading));
    // autonChooser.addOption("Full Field Box Vector",
    //     new CartesianVectorProfileToPointCommand(new Translation2d(15.5, 0.5), drivetrain::getTranslation, 1.5,
    //         Units.inchesToMeters(80), autoDrive, heading)
    //         .andThen(new CartesianVectorProfileToPointCommand(new Translation2d(15.5, 7.5), drivetrain::getTranslation,
    //             1.5, Units.inchesToMeters(80), autoDrive, heading))
    //         .andThen(new CartesianVectorProfileToPointCommand(new Translation2d(0.5, 7.5), drivetrain::getTranslation,
    //             1.5, Units.inchesToMeters(80), autoDrive, heading))
    //         .andThen(new CartesianVectorProfileToPointCommand(new Translation2d(0.5, 0.5), drivetrain::getTranslation,
    //             1.5, Units.inchesToMeters(80), autoDrive, heading))
    //         .andThen(new CartesianVectorProfileToPointCommand(new Translation2d(15, 7), drivetrain::getTranslation, 1.5,
    //             Units.inchesToMeters(80), autoDrive, heading)));
    autonChooser.addOption("Charge Station Test", new blueStartMiddleMiddleBalance(autoDrive, drivetrain, heading));
    autonChooser.addOption("Blue Lefty Left Score 2 Balance", new blueStartLeftyLeftScoreC1GToppyScoreC2Balance(autoDrive, drivetrain, elbow, heading, intake, intakespinner, shoulder));
    autonChooser.addOption("Test", new DriveTest(autoDrive, drivetrain, elbow, heading, intake, intakespinner, shoulder));
    autonChooser.addOption("Partner Showcase ", new PartnerShowcase(autoDrive, drivetrain, elbow, heading, intake, intakespinner, shoulder));
    autonChooser.addOption("Red Lefty Left Score 3", new redLeftyLeftScoreC1GToppyScoreO1GTopScoreC2(autoDrive, drivetrain, elbow, heading, intake, intakespinner, shoulder));

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
    // SmartDashboard.putData(CommandScheduler.getInstance());
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
    JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    JoystickButton driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    
    triggerDriverRight.onTrue(new MaintainHeadingCommand(0, heading));
    triggerDriverLeft.onTrue(new InstantRelocalizeCartesianCommand(drivetrain, vision));

    driverA.whileTrue(new SelectCommand(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(CommandSelector.ONE,  new ConditionalCommand(new BlueConstructTeleopAutoCommand1(autoDrive, drivetrain, heading),
              new RedConstructTeleopAutoCommand1(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue)),
              Map.entry(CommandSelector.TWO,  new ConditionalCommand(new BlueConstructTeleopAutoCommand2(autoDrive, drivetrain, heading),
              new RedConstructTeleopAutoCommand2(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue)),
              Map.entry(CommandSelector.THREE,  new ConditionalCommand(new BlueConstructTeleopAutoCommand3(autoDrive, drivetrain, heading),
              new RedConstructTeleopAutoCommand3(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue))),
          this::selectTeleopAuto));

    driverLeftBumper.whileTrue(new ConditionalCommand(new BlueTeleopAutoLeftSubstation(autoDrive, drivetrain, heading),
      new  RedTeleopAutoLeftSubstation(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue));
    driverRightBumper.whileTrue(new ConditionalCommand(new  BlueTeleopAutoRightSubstation(autoDrive, drivetrain, heading),
      new  RedTeleopAutoRightSubstation(autoDrive, drivetrain, heading), drivetrain::isAllianceBlue));
    
    /** Operator Controller * */
    // Note: Left X axis is used by DeliveryOverrideCommand

    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);
    JoystickButton operatorRightStick = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);
    JoystickButton operatorLeftStick = new JoystickButton(operatorController, XboxController.Button.kLeftStick.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    Trigger triggerOperatorRight = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5);
    Trigger triggerOperatorLeft = new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5);
    Trigger operatorPOVUp = new Trigger(() -> operatorController.getPOV() == 0);
    Trigger operatorPOVDown = new Trigger(() -> operatorController.getPOV() == 180);
    JoystickButton operatorBack = new JoystickButton(operatorController, XboxController.Button.kBack.value);
    JoystickButton operatorStart = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    

    triggerOperatorRight.whileTrue(new IntakeCommand(intake, this, shoulder, elbow, intakespinner));
    triggerOperatorLeft.whileTrue(new OuttakeCommand(intake, this));
    operatorRightBumper.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.SUBSTATION, elbow, shoulder, intakespinner, this));
    operatorLeftBumper.whileTrue(new ArmSetpointWithEnding(Constants.Arm.ArmPosition.CARRYINTERMEDIATE, elbow, shoulder, intakespinner, this).withTimeout(1.5).andThen(new ArmSetpointCommand(Constants.Arm.ArmPosition.CARRY, elbow, shoulder, intakespinner, this)));

    operatorY.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.SCOREHIGH, elbow, shoulder, intakespinner, this));
    operatorB.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.SCOREMID, elbow, shoulder, intakespinner, this));
    operatorA.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.SCORELOW, elbow, shoulder, intakespinner, this));
 
    operatorStart.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.TELEFALLINGCONE, elbow, shoulder, intakespinner, this));
    operatorBack.whileTrue(new ArmSetpointCommand(Constants.Arm.ArmPosition.TELESTANDINGCONE, elbow, shoulder, intakespinner, this));

    operatorPOVUp.onTrue(new IntakeSpinnerAdjustment(intakespinner, 2));
    operatorPOVDown.onTrue(new IntakeSpinnerAdjustment(intakespinner, -2));
    
    operatorRightStick.onTrue(new InstantCommand(()-> setGamePiece(GamePiece.Cone)));
    operatorLeftStick.onTrue(new InstantCommand(()-> setGamePiece(GamePiece.Cube)));

    operatorX.whileTrue(new ArmJoystickCommand(elbow, shoulder, operatorController, ()->getYellowSwitchStatus()));

    // TODO: Create switch to flip between orange and blue
    JoystickButton yellowSwitch = new JoystickButton(operatorStation, 4);
    JoystickButton yellowButton = new JoystickButton(operatorStation, 10);
    JoystickButton purpleButton = new JoystickButton(operatorStation, 11);

  }



  public void instantiateSubsystemsTeleop() {
  }
  public void setGamePiece(GamePiece gamePiece){
    this.gamePiece = gamePiece;
  }
  public GamePiece getGamepiece() {
    return gamePiece;
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

}
