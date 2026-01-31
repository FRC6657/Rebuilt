package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the robot gyroscope. Supports high-frequency yaw sampling for
 * odometry and standard yaw reading.
 */
public interface GyroIO {

  /** Logged gyro sensor inputs including high-frequency odometry samples. */
  @AutoLog
  public static class GyroIOInputs {
    public Rotation2d yawPosition = new Rotation2d(); // Normalized yaw (wraps at 360)
    public double yaw = 0.0; // Raw cumulative yaw in degrees
    public double yawVelocityRadPerSec = 0.0;
    public double yawTimestamp = 0.0; // Timestamp of the latest yaw reading
    public double[] yawTimestamps = new double[] {}; // High-frequency sample timestamps
    public Rotation2d[] yawPositions = new Rotation2d[] {}; // High-frequency yaw samples
  }

  /** Reads the latest gyro values and high-frequency odometry samples. */
  public default void updateInputs(GyroIOInputs inputs) {}

  /** Resets the gyro yaw to the given heading. */
  public default void setYaw(Rotation2d yaw) {}
}
