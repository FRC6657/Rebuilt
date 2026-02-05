package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Utility for calculating shoot-on-the-fly (SOTF) corrections. Compensates for robot velocity
 * during shooting by adjusting turret heading, flywheel RPM, and hood angle.
 *
 * <p>Uses Newton's method to find a virtual distance d* such that Vel(d*) equals the required
 * compensated velocity magnitude. Shot parameters (RPM, hood angle) are then read directly from the
 * lookup table at d*, which naturally produces a tuned RPM/hood combination without any manual
 * splitting or scaling.
 *
 * <p>The algorithm:
 *
 * <ol>
 *   <li>Project the shooter's future position using latency compensation
 *   <li>Look up baseline horizontal velocity for the distance to goal
 *   <li>Build desired velocity vector toward the goal at baseline speed
 *   <li>Subtract robot velocity to get the compensated velocity v_c
 *   <li>Use Newton's method to find d* where Vel(d*) = |v_c|
 *   <li>Read RPM and hood angle from the LUT at d*
 * </ol>
 *
 * @see <a href="https://blog.eeshwark.com/blog/shooting-on-the-fly">Shooting on the Fly</a>
 * @see <a href="https://blog.eeshwark.com/blog/shooting-on-the-fly-pt2">Shooting on the Fly Pt.
 *     2</a>
 */
public class ShootOnTheFlyUtil {

  private static final int MAX_ITERATIONS = 10;
  private static final double CONVERGENCE_THRESHOLD = 0.005; // m/s
  private static final double EPSILON = 0.001; // for numerical derivative

  /** The result of a shoot-on-the-fly calculation. */
  public static class AdjustedShotParameters {

    private final Rotation2d turretHeadingFieldRelative;
    private final double flywheelRPM;
    private final double hoodAngleDeg;
    private final double effectiveDistanceMeters;
    private final double virtualDistanceMeters;

    /**
     * @param turretHeadingFieldRelative the field-relative direction the turret should point
     * @param flywheelRPM the adjusted flywheel speed in RPM
     * @param hoodAngleDeg the adjusted hood angle in degrees
     * @param effectiveDistanceMeters the actual distance from projected position to goal
     * @param virtualDistanceMeters the virtual distance found by Newton's method (LUT input)
     */
    public AdjustedShotParameters(
        Rotation2d turretHeadingFieldRelative,
        double flywheelRPM,
        double hoodAngleDeg,
        double effectiveDistanceMeters,
        double virtualDistanceMeters) {
      this.turretHeadingFieldRelative = turretHeadingFieldRelative;
      this.flywheelRPM = flywheelRPM;
      this.hoodAngleDeg = hoodAngleDeg;
      this.effectiveDistanceMeters = effectiveDistanceMeters;
      this.virtualDistanceMeters = virtualDistanceMeters;
    }

    /** The field-relative direction the turret should point. */
    public Rotation2d getTurretHeadingFieldRelative() {
      return turretHeadingFieldRelative;
    }

    /** The adjusted flywheel speed in RPM. */
    public double getFlywheelRPM() {
      return flywheelRPM;
    }

    /** The adjusted hood angle in degrees. */
    public double getHoodAngleDeg() {
      return hoodAngleDeg;
    }

    /** The actual distance from the projected future position to the goal in meters. */
    public double getEffectiveDistanceMeters() {
      return effectiveDistanceMeters;
    }

    /**
     * The virtual distance found by Newton's method. This is the distance fed into the LUT to
     * produce the returned RPM and hood angle. Differs from effective distance when the robot is
     * moving.
     */
    public double getVirtualDistanceMeters() {
      return virtualDistanceMeters;
    }
  }

  private ShootOnTheFlyUtil() {}

  /**
   * Calculates adjusted shot parameters to compensate for robot motion. Uses Newton's method to
   * find a virtual distance where the LUT's horizontal velocity matches the required compensated
   * velocity, then reads RPM and hood angle from the LUT at that distance.
   *
   * <p>The turret heading is returned in field-relative coordinates. Use {@code
   * Superstructure.getRelativeTurretHeading()} to convert to a turret-relative setpoint.
   *
   * @param shooterFieldPosition the shooter/turret position in field coordinates (meters)
   * @param fieldRelativeVelocity the robot's velocity in field-relative coordinates
   * @param goalPosition the target position in field coordinates (meters)
   * @param shotMap the empirically calibrated shot parameter lookup table
   * @param latencyCompensationSec total latency to compensate for (seconds). Includes camera
   *     processing, network delay, motor spin-up, and approximate ball flight time. Start at 0.1s
   *     and increase if shots land behind the target, decrease if ahead.
   * @return the adjusted shot parameters, or null if the distance is too small to compute
   */
  public static AdjustedShotParameters calculate(
      Translation2d shooterFieldPosition,
      ChassisSpeeds fieldRelativeVelocity,
      Translation2d goalPosition,
      ShotParameterMap shotMap,
      double latencyCompensationSec) {

    // 1. Project future shooter position using latency compensation
    Translation2d futurePosition =
        new Translation2d(
            shooterFieldPosition.getX()
                + fieldRelativeVelocity.vxMetersPerSecond * latencyCompensationSec,
            shooterFieldPosition.getY()
                + fieldRelativeVelocity.vyMetersPerSecond * latencyCompensationSec);

    // 2. Calculate distance and direction from future position to goal
    Translation2d toGoal = goalPosition.minus(futurePosition);
    double distanceToGoal = toGoal.getNorm();

    if (distanceToGoal < 0.1) {
      return null;
    }

    Rotation2d directionToGoal = toGoal.getAngle();

    // 3. Build the desired velocity vector (pointing at goal, baseline horizontal speed)
    double baselineVelocity = shotMap.getHorizontalVelocity(distanceToGoal);
    Translation2d targetVelocity = new Translation2d(baselineVelocity, directionToGoal);

    // 4. Subtract robot velocity to get compensated velocity
    //    v_c = v_s - v_r
    Translation2d robotVelocity =
        new Translation2d(
            fieldRelativeVelocity.vxMetersPerSecond, fieldRelativeVelocity.vyMetersPerSecond);
    Translation2d compensatedVelocity = targetVelocity.minus(robotVelocity);

    Rotation2d correctedHeading = compensatedVelocity.getAngle();
    double requiredSpeed = compensatedVelocity.getNorm();

    // 5. Newton's method: find virtual distance d* where Vel(d*) = requiredSpeed
    double virtualDistance = distanceToGoal;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double currentVelocity = shotMap.getHorizontalVelocity(virtualDistance);

      if (Math.abs(currentVelocity - requiredSpeed) < CONVERGENCE_THRESHOLD) {
        break;
      }

      // Numerical derivative: Vel'(d) ≈ (Vel(d+ε) - Vel(d-ε)) / 2ε
      double velLow = shotMap.getHorizontalVelocity(virtualDistance - EPSILON);
      double velHigh = shotMap.getHorizontalVelocity(virtualDistance + EPSILON);
      double velDeriv = (velHigh - velLow) / (2.0 * EPSILON);

      if (Math.abs(velDeriv) < 1e-6) {
        break;
      }

      virtualDistance -= (currentVelocity - requiredSpeed) / velDeriv;
    }

    // 6. Read shot parameters from LUT at the virtual distance
    ShotParameters result = shotMap.get(virtualDistance);

    return new AdjustedShotParameters(
        correctedHeading,
        result.getFlywheelRPM(),
        result.getHoodAngleDeg(),
        distanceToGoal,
        virtualDistance);
  }
}
