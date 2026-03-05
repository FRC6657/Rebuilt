package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the vision subsystem, including pose filtering thresholds, standard deviation
 * tuning, field layout, and camera definitions.
 */
public class VisionConstants {

  // Filtering thresholds for rejecting bad pose estimates
  public static double maxAmbiguity = 0.3; // Max allowed ambiguity for single-tag estimates
  public static double maxZError = 0.15; // Max allowed Z offset from ground plane (meters)

  // Standard deviation baselines at 1 meter distance with 1 tag.
  // Scaled automatically by (distance^2 / tagCount) at runtime.
  public static double linearStdDevBaseline = 0.04; // Meters
  public static double angularStdDevBaseline = 1; // Radians (rotation ignored)

  // Per-camera standard deviation multipliers (lower = more trusted)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, 1.0
      };

  /** The 2026 Rebuilt field AprilTag layout. */
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  /** Configuration for a single AprilTag camera (name, mount transform, FOV, resolution). */
  public static class CameraInfo {

    public String cameraName;
    public Transform3d robotToCamera; // Transform from robot center to camera
    public Rotation2d diagFOV; // Diagonal field of view
    public int[] cameraRes; // [width, height] in pixels

    public CameraInfo(
        String cameraName, Transform3d robotToCamera, Rotation2d diagFOV, int[] cameraRes) {
      this.cameraName = cameraName;
      this.robotToCamera = robotToCamera;
      this.diagFOV = diagFOV;
      this.cameraRes = cameraRes;
    }
  }

  // Camera definitions (transforms are placeholders - update with real CAD values)
  public static CameraInfo Black1 =
      new CameraInfo(
          "6657-9291-4", // intentional typo
          new Transform3d(
              new Translation3d(0.253248, -0.331830, 0.229592),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(240))),
          Rotation2d.fromDegrees(80),
          new int[] {1280, 800});

  public static CameraInfo Black2 =
      new CameraInfo(
          "6657-9281-3",
          new Transform3d(
              new Translation3d(0.331989, -0.254149, 0.229592),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-10))),
          Rotation2d.fromDegrees(80),
          new int[] {1280, 800});

  public static CameraInfo White1 =
      new CameraInfo(
          "6657-9281-2",
          new Transform3d(
              new Translation3d(0.253248, 0.331830, 0.229592),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-240))),
          Rotation2d.fromDegrees(80),
          new int[] {1280, 800});

  public static CameraInfo White2 =
      new CameraInfo(
          "6657-9281-1",
          new Transform3d(
              new Translation3d(0.331989, 0.254149, 0.229592),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(10))),
          Rotation2d.fromDegrees(80),
          new int[] {1280, 800});
}
