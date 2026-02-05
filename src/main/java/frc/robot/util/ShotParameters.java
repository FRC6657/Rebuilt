package frc.robot.util;

/**
 * Represents the parameters for a shot at a specific distance. These values are typically
 * determined empirically through testing and stored in a {@link ShotParameterMap}.
 */
public class ShotParameters {

  private final double flywheelRPM;
  private final double hoodAngleDeg;

  /**
   * @param flywheelRPM the flywheel speed in rotations per minute
   * @param hoodAngleDeg the hood angle in degrees
   */
  public ShotParameters(double flywheelRPM, double hoodAngleDeg) {
    this.flywheelRPM = flywheelRPM;
    this.hoodAngleDeg = hoodAngleDeg;
  }

  public double getFlywheelRPM() {
    return flywheelRPM;
  }

  public double getHoodAngleDeg() {
    return hoodAngleDeg;
  }
}
