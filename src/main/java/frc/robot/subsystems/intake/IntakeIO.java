package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeConstants.Extension.ExtensionSetpoint;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the intake subsystem. Implementations handle extension
 * position control and roller voltage control.
 */
public interface IntakeIO {

  /** Logged sensor inputs for the intake extension and roller motors. */
  @AutoLog
  public static class IntakeIOInputs {

    public double extensionPosition = 0.0; // Inches
    public double extensionVelocity = 0.0; // Inches per second
    public double extensionAcceleration = 0.0; // Inches per second per second
    public double extensionTemp; // Celsius
    public double extensionVoltage = 0.0; // Volts
    public double extensionStatorCurrent = 0.0; // Amps

    public double rollerTemp = 0.0; // Celsius
    public double rollerVoltage = 0.0; // Volts
    public double rollerStatorCurrent = 0.0; // Amps
  }

  /** Reads the latest sensor values and applies motor outputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets the extension mechanism to the given position/velocity/acceleration profile. */
  public default void changeSetpoint(ExtensionSetpoint setpoint) {}

  /** Sets the roller motor to the given voltage setpoint. */
  public default void changeSetpoint(double setpoint) {}

  /**
   * @return true if the extension is within tolerance of its target position
   */
  public default boolean atSetpoint() {
    return false;
  }
}
