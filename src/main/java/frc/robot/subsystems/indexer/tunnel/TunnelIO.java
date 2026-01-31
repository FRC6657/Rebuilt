package frc.robot.subsystems.indexer.tunnel;

import frc.robot.subsystems.indexer.tunnel.TunnelConstants.TunnelSetpoint;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction interface for the tunnel indexer motor. */
public interface TunnelIO {

  /** Logged sensor inputs for the tunnel motor. */
  @AutoLog
  public static class TunnelIOInputs {
    public double temp = 0.0; // Celsius
    public double voltage = 0.0; // Volts
    public double statorCurrent = 0.0; // Amps
  }

  /** Reads the latest sensor values and applies the motor output. */
  public default void updateInputs(TunnelIOInputs inputs) {}

  /** Sets the tunnel roller to the given voltage setpoint. */
  public default void changeSetpoint(TunnelSetpoint setpoint) {}
}
