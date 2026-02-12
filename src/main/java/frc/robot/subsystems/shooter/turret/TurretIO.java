package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction interface for the turret rotation mechanism. */
public interface TurretIO {

  /** Logged sensor inputs for the turret motor. */
  @AutoLog
  public static class TurretIOInputs {
    public double position = 0.0; // Degrees
    public double velocity = 0.0; // Degrees per second
    public double acceleration = 0.0; // Degrees per second per second
    public double temp = 0.0; // Celsius
    public double voltage = 0.0; // Volts
    public double statorCurrent = 0.0; // Amps
  }

  /** Reads the latest sensor values and applies the position control output. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Sets the turret to the target heading in degrees (wrapped to 0-360).
   * This method will tell the turret to rotate to the correct angle optimizing for minimal motion while also keeping the position within tolerance.
   * @param setpoint the target angle relative to the turret.
   */
  public default void changeSetpoint(double setpoint) {}

  /** Sets the turret heading with a velocity feedforward for smoother tracking while moving. */
  public default void changeSetpoint(double setpoint, double feedforwardDegPerSec) {
    changeSetpoint(setpoint);
  }

  /**
   * @return true if the turret is within tolerance of its target heading
   */
  public default boolean atSetpoint() {
    return false;
  }
}
