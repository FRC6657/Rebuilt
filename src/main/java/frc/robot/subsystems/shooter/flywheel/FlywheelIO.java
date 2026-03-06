package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the flywheel shooter. Tracks both leader and follower motor
 * telemetry.
 */
public interface FlywheelIO {

  /** Logged sensor inputs for the flywheel leader and follower motors. */
  @AutoLog
  public static class FlywheelIOInputs {

    public double velocity = 0.0; // RPM
    public double acceleration = 0.0; // RPM per second

    public double leaderTemp = 0.0; // Celsius
    public double leaderVoltage = 0.0; // Volts
    public double leaderStatorCurrent = 0.0; // Amps

    public double followerTemp = 0.0; // Celsius
    public double followerVoltage = 0.0; // Volts
    public double followerStatorCurrent = 0.0; // Amps
  }

  /** Reads the latest sensor values and applies motor outputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Sets the flywheel target velocity in RPM. */
  public default void changeSetpoint(double setpoint) {}

  /**
   * @return true if the flywheel is within velocity tolerance of its target
   */
  public default boolean atSetpoint() {
    return false;
  }
}
