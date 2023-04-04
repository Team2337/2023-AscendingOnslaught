package frc.robot;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotType {

  public enum Type {
    COMPETITION("Comp"),
    SKILLSBOT("Skills"),
    PRACTICE("Practice");

    public final String description;

    private Type(String description) {
      this.description = description;
    }
  }

  private static String kPracticeBotMAC = "00:80:2F:17:89:85";

  private static String kSkillsBotMAC = "00:80:2F:30:2E:F2";

  public static Type getRobotType() {
    String mac = getMACAddress();
    if (mac != null && mac.equals(kPracticeBotMAC)) {
      return Type.PRACTICE;
    } else if (mac != null && mac.equals(kSkillsBotMAC)) {
      return Type.SKILLSBOT;
    }
    return Type.COMPETITION;
  }

  public static String getMACAddress() {
    try {
      NetworkInterface network = NetworkInterface.getByInetAddress(InetAddress.getLocalHost());
      byte[] address = network.getHardwareAddress();
      StringBuilder sb = new StringBuilder();
      for (int i = 0; i < address.length; i++) {
        sb.append(String.format("%02X%s", address[i], (i < address.length - 1) ? ":" : ""));
      }

      // Remove once validated.
      SmartDashboard.putString("Mac-Address", sb.toString());
      return sb.toString();
    } catch (UnknownHostException | SocketException | NullPointerException e) {
      e.printStackTrace();
    }
    return null;
  }
}