package frc.robot.subsystems;

import java.lang.ref.WeakReference;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.interfaces.AutoDrivableCommand;

/**
 * The AutoDrive subsystem is used to proxy values between a Command to
 * the SwerveDriveCommand. This allow us to automatically move the robot
 * in both autonomous and teleop in the same way, with the added benifit
 * of being able to control the robot with the Heading subsystem through
 * the SwerveDriveCommand without needing to re-implement this
 * functionality in every Command.
 *
 * The `calculate` method should return the values the robot should move
 * (forward, strafe, and if it's field oriented) and will pass the driver
 * input. The Command is responsible for resolving driver input with the
 * calculated output, if applicable. This method can return null if the
 * Command does not wish to supply any movement values.
 *
 * In order to use the AutoDrive subsystem -
 * 1) `implement the `AutoDrivableCommand` in your Command
 *
 * public class MyCommand extends CommandBase implements AutoDrivableCommand {
 *   public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
 *     return ...;
 *   }
 * }
 *
 * 2) `require` the AutoDrive subsystem. Only one Command should be
 * providing values to the AutoDrive subsystem at a time.
 *
 * @Override
 * public void initialize() {
 *   autoDrive.setDelegate(this);
 * }
 *
 * 3) Generate an output value in your Command's `periodic` function.
 * 4) Return an `AutoDrive.State` from the `calculate` method that negotiates
 * driver input values calculated from your `periodic` function. Ex: If a
 * calculated forward value is higher than the driver's forward value, consider
 * slowing your forward output to be the requested forward output from the driver.
 * This method can return null if the Command does not wish to provide any
 * movement values for a given cycle.
 * 5) Call `clearDelegate` in your Command's `end` function
 *
 * @Override
 * public void end() {
 *   autoDrive.clearDelegate();
 * }
 */
public class AutoDrive extends SubsystemBase {

  public AutoDrive() {
  }

  private WeakReference<AutoDrivableCommand> delegateReference;

  public static class State {
    public double forward;
    public double strafe;
    // Auto drive commands should ALWAYS generate robot-centric moves
    public final boolean isFieldOriented = false;

    public State(double forward, double strafe) {
      this.forward = forward;
      this.strafe = strafe;
    }
  }

  /**
   * Registered an AutoDrivableCommand to be called when an output is needed for
   * the SwerveDriveCommand. Note: Only one AutoDrivableCommand can be registered
   * at a time. Commands calling setDelegate should require
   * the AutoDrive subsystem in order to prevent other commands from
   * registering for callbacks.
   *
   * @param command
   */
  public void setDelegate(AutoDrivableCommand command) {
    this.delegateReference = new WeakReference<AutoDrivableCommand>(command);
  }

  public void clearDelegate() {
    this.delegateReference = null;
  }

  /**
   * Calculate forward/strafe values using the current command. May return null if
   * there is no auto drive command scheduled or the currently auto drive command
   * does not specify an auto drive state.
   *
   * @param forward         - The user-initiated forward
   * @param strafe          - The user-initiated strafe
   * @param isFieldOriented - The user-initiated isFieldOriented
   * @return - A negotiated forward/strafe from the auto driveable command
   */
  public State calculate(double forward, double strafe, boolean isFieldOriented) {
    AutoDrivableCommand command = getAutoDriveCommand();
    if (command == null) {
      return null;
    }
    return command.calculate(forward, strafe, isFieldOriented);
  }

  private AutoDrivableCommand getAutoDriveCommand() {
    if (delegateReference == null) {
      return null;
    }
    return delegateReference.get();
  }
/*
  private String getAutoDriveCommandName() {
    AutoDrivableCommand command = getAutoDriveCommand();
    if (command == null) {
      return "N/A";
    }
    return command.toString();
  }
*/
  @Override
  public void periodic() {
    /*
    String commandName = getAutoDriveCommandName();
    SmartDashboard.putString("AutoDrive/Command", commandName);
    */
  }

}
