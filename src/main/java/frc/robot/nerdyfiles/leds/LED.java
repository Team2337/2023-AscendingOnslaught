package frc.robot.nerdyfiles.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Setup for addressable LED strip
 * 
 * @author Madison J
 */

public class LED extends SubsystemBase {
	/**
	 * 0-60 are left LEDs
	 * 61-130 are right LEDs
	 */

	private static AddressableLED led;
	private static AddressableLEDBuffer ledBuffer;
	private static int LED_LENGTH = 126;

	/**
	 * Controls the LEDs on the Robot 
	 * 
	 * @param pwm The PWM port that the blinkin is plugged into
	 */
	public LED() {
		led = new AddressableLED(Constants.LEDSTRIP_PWM_ID);
		ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
		led.setLength(ledBuffer.getLength());
	}

	/**
	 * Sets the color of the LEDs
	 * 
	 * @param color A Color object reflecting the color you want to use on the LEDs.  i.e.  kRed, kBlue, kSeashell
	 */
  public void setColor(Color color) {
		for (int i = 0; i < LED_LENGTH; i++) {
			ledBuffer.setLED(i, color);
		}
			led.setData(ledBuffer);
			led.start();
	}

	// maybe swap out 20 to TX_MAX

	public void setColor(Color color, double tx) {
		for (int i = 0; i < LED_LENGTH; i++) {
		if (tx > -Constants.VISION_TOLERANCE && tx < Constants.VISION_TOLERANCE) {
			ledBuffer.setLED(i, color);
		}else if (tx >= Constants.VISION_TOLERANCE){
			if(tx <= (20 - ((double)i * (20/(double)LED_LENGTH*2)))){
				ledBuffer.setLED(i,color);
			}else{
				ledBuffer.setRGB(i, 0, 0, 0);
			}
		}else if (tx <= -Constants.VISION_TOLERANCE){
			if(tx >= (20-(20/(double)LED_LENGTH*2) - ((double)i * (20/(double)LED_LENGTH*2)))){
				ledBuffer.setLED(i,color);
			}else{
				ledBuffer.setRGB(i,0,0,0);
			}
		}
	}
		led.setData(ledBuffer);
		led.start();
		System.out.println();
	}

	public void setColorRGB(int r, int g, int b, double tx) {
		for (int i = 0; i < LED_LENGTH; i++) {
		if (tx > -Constants.VISION_TOLERANCE && tx < Constants.VISION_TOLERANCE) {
			ledBuffer.setRGB(i, r, g, b);
		}else if (tx >= Constants.VISION_TOLERANCE){
			if(tx <= (20 - ((double)i * (20/(double)LED_LENGTH*2)))){
				ledBuffer.setRGB(i, r, g, b);
			}else{
				ledBuffer.setRGB(i, 0, 0, 0);
			}
		}else if (tx <= -Constants.VISION_TOLERANCE){
			if(tx >= (20-(20/(double)LED_LENGTH*2) - ((double)i * (20/(double)LED_LENGTH*2)))){
				ledBuffer.setRGB(i, r, g, b);
			}else{
				ledBuffer.setRGB(i,0,0,0);
			}
		}
	}
		led.setData(ledBuffer);
		led.start();
	}

	public void setFrontColor(Color color) {
		for (int i = 0; i < LED_LENGTH; i++) {
			if (i > 60 && i < 78) {
				ledBuffer.setLED(i, color);
			}
		}
		led.setData(ledBuffer);
		led.start();
	}

	public void setLeftColor(Color color) {
		for (int i = 0; i < LED_LENGTH; i++) {
			if (i < 61) {
				ledBuffer.setLED(i, color);
			}
		}
		led.setData(ledBuffer);
		led.start();
	}

	public void setRightColor(Color color) {
		for (int i = 0; i < LED_LENGTH; i++) {
			if (i > 60) {
				ledBuffer.setLED(i, color);
			}
		}
		led.setData(ledBuffer);
		led.start();
	}

	public void setColorRGB(int r, int g, int b) {
		for (int i = 0; i < LED_LENGTH; i++) {
			ledBuffer.setRGB(i, r, g, b);
		}
		led.setData(ledBuffer);
		led.start();
	}

  public void setColorLeft(Color color) {
		for (int i = 0; i < 8; i++) {
			ledBuffer.setLED(i, color);
		}
			led.setData(ledBuffer);
			led.start();
	}

    public void setColorLeft5() {
        ledBuffer.setLED(0, Color.kRed);
        ledBuffer.setLED(1, Color.kOrange);
        ledBuffer.setLED(2, Color.kBlue);
        ledBuffer.setRGB(3, 87, 26, 125);
        ledBuffer.setLED(4, Color.kGray);


            led.setData(ledBuffer);
            led.start();
	}

  public void setColorRight(Color color) {
		for (int i = 9; i < 16; i++) {
			ledBuffer.setLED(i, color);
		}
			led.setData(ledBuffer);
			led.start();
	}

  public void setColorEdge(Color color) {
		for (int i = 0; i < 2; i++) {
			ledBuffer.setLED(i, color);
		}
    for (int i = 14; i < 16; i++) {
			ledBuffer.setLED(i, color);
		}
			led.setData(ledBuffer);
			led.start();
	}

  public void setColorMiddle() {
		for (int i = 0; i < LED_LENGTH; i++) {
			if (i > 27 && i < 48) {
				ledBuffer.setRGB(i, 25, 25, 25);
			} else {
				ledBuffer.setRGB(i, 0, 0, 0);
			}
		}
			led.setData(ledBuffer);
			led.start();
	}

	public static void setRed() {
		for (int i = 0; i < LED_LENGTH; i++) {
			ledBuffer.setLED(i, Color.kRed);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public void setGreen() {
		for (int i = 0; i < LED_LENGTH; i++) {
			ledBuffer.setLED(i, Color.kGreen);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public static void setBlue() {
		for (int i = 0; i < LED_LENGTH; i++) {
			ledBuffer.setLED(i, Color.kBlue);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public void setYellow() {
		for (int i = 0; i < LED_LENGTH; i++) {
			ledBuffer.setLED(i, Color.kYellow);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public void setSeashell() {
		for (int i = 0; i < LED_LENGTH; i++) {
			ledBuffer.setLED(i, Color.kSeashell);
		}
			led.setData(ledBuffer);
			led.start();
	}

  
	public static void setLeftOff() {
		for (int i = 0; i < 8; i++) {
			ledBuffer.setRGB(i, 0, 0, 0);
		}
			led.setData(ledBuffer);
			led.start();
  }

  
	public static void setRightOff() {
		for (int i = 9; i < 16; i++) {
			ledBuffer.setRGB(i, 0, 0, 0);
		}
			led.setData(ledBuffer);
			led.start();
  }

	public static void setOff() {
		for (int i = 0; i < LED_LENGTH; i++) {
			ledBuffer.setRGB(i, 0, 0, 0);
		}
			led.setData(ledBuffer);
			led.start();
  }
} 