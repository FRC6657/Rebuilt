package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for an AprilTag camera. Each camera produces pose observations
 * that the vision system uses to update the robot's field-relative position estimate.
 */
public interface ApriltagCameraIO {

  /** Logged inputs for one AprilTag camera. */
  @AutoLog
  public static class ApriltagCameraIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0]; // All tag IDs seen this frame
  }

  /**
   * A single pose estimate derived from one or more AprilTag detections.
   *
   * @param timestamp the FPGA timestamp of the observation
   * @param pose the estimated robot pose in field coordinates
   * @param ambiguity the pose ambiguity (lower is better, only meaningful for single-tag)
   * @param tagCount the number of tags used in this estimate
   * @param averageTagDistance the average distance to detected tags in meters
   */
  public static record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  /** Reads the latest camera results and converts them to pose observations. */
  public default void updateInputs(ApriltagCameraIOInputs inputs) {}
}
