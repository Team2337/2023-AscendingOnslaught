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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.DriverDashboardPositions;
import frc.robot.commands.CartesianHeadingToTargetCommand;
import frc.robot.commands.arm.ArmDemoCommand;
import frc.robot.commands.arm.ArmJoystickCommand;
import frc.robot.commands.arm.ArmSetpointCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.auto.teleop.BlueConstructTeleopAutoCommand;
import frc.robot.commands.auto.teleop.RedConstructTeleopAutoCommand;
import frc.robot.commands.auto.test.AngleTest;
import frc.robot.commands.auto.test.MoveForwardTest;
import frc.robot.commands.auto.test.Test;
import frc.robot.commands.swerve.MaintainHeadingCommand;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.commands.vision.InstantRelocalizeCartesianCommand;
import frc.robot.commands.vision.InstantRelocalizeCommand;
import frc.robot.commands.vision.LimelightHeadingAndInstantRelocalizeCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);

  private final PigeonIMU pigeon = new PigeonIMU(0);

  private final AutoDrive autoDrive = new AutoDrive();
  private final Drivetrain drivetrain = new Drivetrain(pigeon);
  private final Vision vision = new Vision();
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation, drivetrain::isMoving);
  private final Arm arm = new Arm();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final SendableChooser<String> startingPosChooser = new SendableChooser<>();
  private final SendableChooser<Double> startingAngleChooser = new SendableChooser<>();

  public RobotContainer() {
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));
    heading.setDefaultCommand(
        new CartesianHeadingToTargetCommand(drivetrain::getTranslation, operatorLeftBumper::getAsBoolean, driverRightBumper::getAsBoolean, drivetrain, heading, vision));
    // vision.setDefaultCommand(new PeriodicRelocalizeCartesian(drivetrain, vision));
    //arm.setDefaultCommand(new ArmBasicJoystickCommand(arm, () -> operatorController));
    // Configure the button bindings
    configureButtonBindings();

    // Create auton selector
    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());
    autonChooser.addOption("Test", new Test(autoDrive, drivetrain, heading));
    autonChooser.addOption("Angle Test", new AngleTest(autoDrive, drivetrain, heading));
    autonChooser.addOption("Move Test", new blueRightMiddleToBottom(autoDrive, drivetrain, heading));
    autonChooser.addOption("Move Forward Test", new MoveForwardTest(autoDrive, drivetrain, heading));
    autonChooser.addOption("Vector Test", new vectorBlueRightMiddleToBottom(autoDrive, drivetrain, heading));
    autonChooser.addOption("Full Field Straight Vector", new CartesianVectorProfileToPointCommand(new Translation2d(16,0), drivetrain::getTranslation, 1.5, Units.inchesToMeters(80), autoDrive, heading));
    autonChooser.addOption("Full Field Straight XY", new CartesianProfiledPointToPointCommand(new Translation2d(16,0), drivetrain::getTranslation, drivetrain::getRotation, 1.5, 1.5, Units.inchesToMeters(80), Units.inchesToMeters(80), autoDrive, heading));
    autonChooser.addOption("Full Field Diagonal Vector", new CartesianVectorProfileToPointCommand(new Translation2d(16,8), drivetrain::getTranslation, 1.5, Units.inchesToMeters(80), autoDrive, heading));
    autonChooser.addOption("Full Field Diagonal XY", new CartesianProfiledPointToPointCommand(new Translation2d(16,8), drivetrain::getTranslation, drivetrain::getRotation, 1.5, 1.5, Units.inchesToMeters(80), Units.inchesToMeters(80), autoDrive, heading));
    autonChooser.addOption("Full Field Box Vector",
      new CartesianVectorProfileToPointCommand(new Translation2d(15.5,0.5), drivetrain::getTranslation, 1.5, Units.inchesToMeters(80), autoDrive, heading)
      .andThen(new CartesianVectorProfileToPointCommand(new Translation2d(15.5,7.5), drivetrain::getTranslation, 1.5, Units.inchesToMeters(80), autoDrive, heading))
      .andThen(new CartesianVectorProfileToPointCommand(new Translation2d(0.5,7.5), drivetrain::getTranslation, 1.5, Units.inchesToMeters(80), autoDrive, heading))
      .andThen(new CartesianVectorProfileToPointCommand(new Translation2d(0.5,0.5), drivetrain::getTranslation, 1.5, Units.inchesToMeters(80), autoDrive, heading))
      .andThen(new CartesianVectorProfileToPointCommand(new Translation2d(15,7), drivetrain::getTranslation, 1.5, Units.inchesToMeters(80), autoDrive, heading))
    );
    autonChooser.addOption("Charge Station Test", new blueStartMiddleMiddleBalance(autoDrive, drivetrain, heading));

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
/* 
  public void resetRobot() {
    // Other option here is Constants.STARTING_ANGLE for booting against Hub
    pigeon.setYaw(0, 250);
    drivetrain.resetPosition(
        new Pose2d(
            Constants.Auto.kStartAtZero.toFieldCoordinate(),
            drivetrain.getGyroscopeRotation()));
  }
*/
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


  public void resetRobotChooser(String startPos, double startingAngle) {
    if (startPos == "0.5,0.5") {
      drivetrain.resetPosition(new Pose2d(new Translation2d(0.5,0.5), Rotation2d.fromDegrees(startingAngle)));
      return;
    }
    if (DriverStation.getAlliance() == Alliance.Blue) {
      switch (startPos) {

        case "Zero":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.zeroPoint, Rotation2d.fromDegrees(startingAngle)));
          break;
        
        case "Right Right":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.blueGridRightRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Middle":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.blueGridRightRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Left":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.blueGridRightRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Right":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.blueGridMiddleRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Middle":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.blueGridMiddleRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Left":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.blueGridMiddleRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Right":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.blueGridLeftRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Middle":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.blueGridLeftRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Left":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.blueGridLeftRobotLeft, Rotation2d.fromDegrees(startingAngle)));
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
          drivetrain.resetPosition(new Pose2d(Constants.Auto.redGridRightRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Middle":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.redGridRightRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Right Left":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.redGridRightRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Right":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.redGridMiddleRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Middle":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.redGridMiddleRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Middle Left":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.redGridMiddleRobotLeft, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Right":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.redGridLeftRobotRight, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Middle":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.redGridLeftRobotCenter, Rotation2d.fromDegrees(startingAngle)));
          break;

        case "Left Left":
          drivetrain.resetPosition(new Pose2d(Constants.Auto.redGridLeftRobotLeft, Rotation2d.fromDegrees(startingAngle)));
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
    SmartDashboard.putData(CommandScheduler.getInstance());
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

    driverStart.onTrue(new MaintainHeadingCommand(0, heading));
    driverA.whileTrue(new ConditionalCommand(new BlueConstructTeleopAutoCommand(autoDrive, drivetrain, heading, this), new RedConstructTeleopAutoCommand(autoDrive, drivetrain, heading, this), drivetrain::isAllianceBlue));
    triggerDriverLeft.onTrue(new InstantCommand(() -> drivetrain.setTeleopAutoPosition(10)));   
    triggerDriverRight.onTrue(new InstantCommand(() -> drivetrain.setTeleopAutoPosition(7)));
    


    // driverLeftBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.BLUE,
    // delivery, kicker));
    // driverRightBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.RED,
    // delivery, kicker));

    driverBack.whenPressed(new InstantRelocalizeCommand(drivetrain, vision));

    driverX.onTrue(new InstantRelocalizeCartesianCommand(drivetrain, vision));



    /** Operator Controller * */
    // Note: Left X axis is used by DeliveryOverrideCommand

    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);
    JoystickButton operatorRightStick = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);
    JoystickButton operatorLeftStick = new JoystickButton(operatorController, XboxController.Button.kLeftStick.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    Trigger triggerOperatorRight = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5);
    Trigger triggerOperatorLeft = new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5);
    
    // Operator left bumper used for vision tracking by default commands.
    // Operator right bumper below in the configureButtonBindingsTeleop() method.
    JoystickButton operatorBack = new JoystickButton(operatorController, XboxController.Button.kBack.value);
    JoystickButton operatorStart = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    JoystickButton yellowSwitch = new JoystickButton(operatorStation, 4);
    JoystickButton yellowButton = new JoystickButton(operatorStation, 8);
    JoystickButton blueButton = new JoystickButton(operatorStation, 9);

    operatorRightStick.whileHeld(new LimelightHeadingAndInstantRelocalizeCommand(drivetrain, heading, vision));


    operatorB.whileTrue(new ArmSetpointCommand(arm, -2000, 46000));
    //90,0
    operatorX.whileTrue(new ArmSetpointCommand(arm, -13000, -27000));
    //0, 90
    operatorY.whileTrue(new ArmSetpointCommand(arm, 5500, 28000));

    operatorBack.whileTrue(new ArmSetpointCommand(arm, -Constants.SHOULDER_OFFSET_FOR_PREMADE_SETPOINTS_IN_TICKS , -Constants.ELBOW_OFFSET_FOR_PREMADE_SETPOINTS_IN_TICKS));
    //operatorController.povUp().whileTrue(new ArmSetpointCommand(arm, -12500, -70500));
    
    operatorStart.onTrue(new ArmDemoCommand(arm));


    
    operatorA.whileTrue(new ArmJoystickCommand(arm, operatorController));
    //operatorLeftBumper().whileTrue(new ArmSetpointCommand(arm, -13000, -27000));
    operatorRightBumper.whileTrue(new ArmSetpointCommand(arm, -40000, 17500));
    /** Driverstation Controls * */
    //TODO: Create switch to flip between orange and blue
  }

  public void instantiateSubsystemsTeleop() {
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

  public boolean getYellowSwitchStatus() {
    return operatorStation.yellowSwitch.getAsBoolean();
  }

  public boolean getClearSwitchStatus() {
    return operatorStation.clearSwitch.getAsBoolean();
  }

  public boolean getDriverInput() {
    return (Math.abs(driverController.getRightX()) > 0.2) || (Math.abs(driverController.getLeftY()) > 0.2);
  }

}
