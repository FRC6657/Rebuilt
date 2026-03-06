package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for a single swerve module (drive + turn motors). Includes
 * high-frequency odometry data from the PhoenixOdometryThread.
 */
public interface ModuleIO {

  /** Logged sensor inputs for one swerve module. */
  @AutoLog
  public static class ModuleIOInputs {

    public String name = ""; // Human-readable module name (e.g. "Front Left")
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveOutputVolts = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;

    public Rotation2d turnAbsolutePosition = new Rotation2d(); // From absolute encoder
    public Rotation2d turnPosition = new Rotation2d(); // From relative encoder (motor)
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    // High-frequency odometry data from the odometry thread
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositions = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Reads the latest sensor values from hardware. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Sets the drive motor speed in closed-loop mode. */
  public default void changeDriveSetpoint(double metersPerSecond) {
    changeDriveSetpoint(metersPerSecond, false);
  }

  /**
   * Sets the drive motor speed.
   *
   * @param metersPerSecond the desired wheel speed
   * @param openLoop if true, uses open-loop voltage; if false, uses closed-loop velocity PID
   */
  public default void changeDriveSetpoint(double metersPerSecond, boolean openLoop) {}

  /** Sets the turn motor to the desired wheel angle. */
  public default void changeTurnSetpoint(Rotation2d rotation) {}

  /** Resets the drive encoder position to zero. */
  public default void resetDriveEncoder() {}
}
