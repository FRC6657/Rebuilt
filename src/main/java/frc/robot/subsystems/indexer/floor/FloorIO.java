package frc.robot.subsystems.indexer.floor;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction interface for the floor indexer motor. */
public interface FloorIO {

  /** Logged sensor inputs for the floor motor. */
  @AutoLog
  public static class FloorIOInputs {
    public double temp = 0.0; // Celsius
    public double voltage = 0.0; // Volts
    public double statorCurrent = 0.0; // Amps
  }

  /** Reads the latest sensor values and applies the motor output. */
  public default void updateInputs(FloorIOInputs inputs) {}

  /** Sets the floor roller to the given voltage setpoint. */
  public default void changeSetpoint(double newSetpoint) {}
}
