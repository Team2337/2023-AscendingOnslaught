package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.nerdyfiles.leds.LED;

/**
 * Sets the color according to various factors, such as if the robot was
 * intaking, climbing, etc.
 * 
 * @author Madison J.
 */
public class LEDOff extends InstantCommand {

	public LEDOff(LED LED) {
		addRequirements(LED);
	}

  @Override
	public void initialize() {
		LED.setOff();
	}
} 