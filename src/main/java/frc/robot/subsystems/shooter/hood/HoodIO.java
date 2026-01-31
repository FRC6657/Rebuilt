package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction interface for the hood angle mechanism. */
public interface HoodIO {

  /** Logged sensor inputs for the hood motor. */
  @AutoLog
  public static class HoodIOInputs {
    public double position = 0.0; // Degrees
    public double temp = 0.0; // Celsius
    public double voltage = 0.0; // Volts
    public double statorCurrent = 0.0; // Amps
  }

  /** Reads the latest sensor values and applies the position control output. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Sets the hood to the target angle in degrees (clamped to min/max). */
  public default void changeSetpoint(double setpoint) {}

  /**
   * @return true if the hood is within tolerance of its target angle
   */
  public default boolean atSetpoint() {
    return false;
  }
}
