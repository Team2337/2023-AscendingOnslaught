package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;

/**
 * @author Michael F
 */
public class PowerDistributionHub extends SubsystemBase {

  private PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public PowerDistributionHub() {
    setupShuffleboard(Constants.DashboardLogging.PDH);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
      ShuffleboardTab pdhTab = Shuffleboard.getTab("PDH");
      ShuffleboardLayout widget = pdhTab.getLayout("Diagnostics", BuiltInLayouts.kList)
        .withSize(6, 12)
        .withPosition(0, 0);
      widget.addNumber("Temperature (F)", () -> {
        return Utilities.convertCelsiusToFahrenheit(getTemperature());
      });
      widget.addNumber("Voltage", this::getVoltage);
      widget.addNumber("Total Current (Amps)", pdh::getTotalCurrent);
      widget.addNumber("Total Energy (Joules)", pdh::getTotalEnergy);
      widget.addNumber("Total Power (Watts)", pdh::getTotalPower);
    }
  }

  @Override
  public void periodic() {}

  /**
   * @param enabled Sets the switchable channel to be enabled/disabled
   */
  public void setSwitchableChannel(boolean enabled) {
    pdh.setSwitchableChannel(enabled);
  }


  // Enables power to port 23 on the PDH
  public void setSwitchableChannelOn() {
    pdh.setSwitchableChannel(true);
  }

  // Disables power to port 23 on the PDH
  public void setSwitchableChannelOff() {
    pdh.setSwitchableChannel(false);
  }

  /**
   * @return Whether or not the switchable channel is disabled
   */
  public boolean getSwitchableChannel() {
    return pdh.getSwitchableChannel();
  }

  /**
   * @return Temperature of PDH in Celsius
   */
  public double getTemperature() {
    return pdh.getTemperature();
  }

  /**
   * @return Voltage of PDH
   */
  public double getVoltage() {
    return pdh.getVoltage();
  }

  /**
   * @param channel The channel to check the current of
   * @return The current of the specified channel in Amperes
   */
  public double getChannelCurrent(int channel) {
    return pdh.getCurrent(channel);
  }

  /**
   * Current for the intake motors in amps
   */
  public double getIntakeCurrent() {
    return getChannelCurrent(Constants.INTAKE_MOTOR_ID);
  }

  /**
   * Current for the delivery motors in amps
   */
  public double getDeliveryCurrent() {
    return getChannelCurrent(Constants.DELIVERY_MOTOR_ID);
  }

}
