package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  public static enum CAN {
    ExtensionMotor(11),
    RollerMotor(12);

    public int id;

    CAN(int id) {
      this.id = id;
    }
  }

  static double minLength = Units.inchesToMeters(0);
  static double maxLength = Units.inchesToMeters(6);
}
