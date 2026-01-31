package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a single swerve module (drive + turn). Wraps a ModuleIO and processes high-frequency
 * odometry samples collected by the PhoenixOdometryThread.
 */
public class Module {

  private ModuleIO io;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  /** High-frequency odometry positions built from odometry thread samples. */
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * @param io the hardware IO implementation (real or simulated)
   */
  public Module(ModuleIO io) {
    this.io = io;
  }

  /**
   * Runs the module with the given state
   *
   * @param state The state to run the module with
   * @return The optimized module state being ran
   */
  public SwerveModuleState runSetpoint(SwerveModuleState state, boolean openLoop) {
    state.optimize(inputs.turnPosition);
    io.changeTurnSetpoint(state.angle);
    io.changeDriveSetpoint(state.speedMetersPerSecond, openLoop);
    return state;
  }

  /**
   * @return the current state of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition);
  }

  /**
   * @return the current position of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
  }

  /** Updates sensor inputs, logs them, and reconstructs high-frequency odometry positions. */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs(
        new StringBuilder("Swerve/").append(inputs.name).append(" Module").toString(), inputs);

    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      odometryPositions[i] =
          new SwerveModulePosition(
              inputs.odometryDrivePositions[i], inputs.odometryTurnPositions[i]);
    }
  }

  /**
   * @return the timestamps of high-frequency odometry samples since last update
   */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /**
   * @return the high-frequency odometry positions corresponding to each timestamp
   */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Reset the drive encoder to 0 */
  public void resetDriveEncoder() {
    io.resetDriveEncoder();
  }
}
