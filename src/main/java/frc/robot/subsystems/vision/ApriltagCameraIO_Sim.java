package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Simulated AprilTag camera using PhotonVision's VisionSystemSim. Extends the real implementation
 * and updates the simulated world before reading results. All camera sims share one static
 * VisionSystemSim instance.
 */
public class ApriltagCameraIO_Sim extends ApriltagCameraIO_Real {

  private static VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * @param cameraInfo the camera configuration (name, transform, FOV, resolution)
   * @param poseSupplier supplies the robot's current pose for the simulation
   */
  public ApriltagCameraIO_Sim(CameraInfo cameraInfo, Supplier<Pose2d> poseSupplier) {
    super(cameraInfo);
    this.poseSupplier = poseSupplier;
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.kTagLayout);
    }
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(cameraInfo.cameraRes[0], cameraInfo.cameraRes[1], cameraInfo.diagFOV);
    cameraProp.setCalibError(0, 0);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setExposureTimeMs(20);
    cameraProp.setFPS(50);
    cameraProp.setLatencyStdDevMs(5.0);
    cameraSim = new PhotonCameraSim(camera, cameraProp, VisionConstants.kTagLayout);
    visionSim.addCamera(cameraSim, cameraInfo.robotToCamera);
  }

  @Override
  public void updateInputs(ApriltagCameraIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
