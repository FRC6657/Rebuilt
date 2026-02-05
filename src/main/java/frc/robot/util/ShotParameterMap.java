package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * A lookup table that maps distance to the goal (in meters) to shot parameters. Uses WPILib's
 * {@link InterpolatingDoubleTreeMap} for linear interpolation between empirically measured data
 * points.
 *
 * <p>Entries should be added during robot initialization with values determined through testing at
 * various distances. The horizontal exit velocity is derived from distance and time of flight:
 * {@code v_horizontal = distance / tof}.
 */
public class ShotParameterMap {

  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  /**
   * Adds a calibrated shot data point at a specific distance.
   *
   * @param distanceMeters the distance to the goal in meters
   * @param flywheelRPM the flywheel speed that scores at this distance
   * @param hoodAngleDeg the hood angle in degrees that scores at this distance
   * @param timeOfFlightSec the measured ball flight time at this distance
   */
  public void addEntry(
      double distanceMeters, double flywheelRPM, double hoodAngleDeg, double timeOfFlightSec) {
    rpmMap.put(distanceMeters, flywheelRPM);
    hoodAngleMap.put(distanceMeters, hoodAngleDeg);
    timeOfFlightMap.put(distanceMeters, timeOfFlightSec);
  }

  /**
   * Retrieves interpolated shot parameters for the given distance.
   *
   * @param distanceMeters the distance to the goal in meters
   * @return interpolated shot parameters for the given distance
   */
  public ShotParameters get(double distanceMeters) {
    return new ShotParameters(rpmMap.get(distanceMeters), hoodAngleMap.get(distanceMeters));
  }

  /**
   * Returns the horizontal velocity for a given distance: {@code distance / Tof(distance)}. Used by
   * Newton's method in the SOTF solver to evaluate Vel(d) at arbitrary distances.
   *
   * @param distanceMeters the distance to evaluate at
   * @return horizontal velocity in meters per second
   */
  public double getHorizontalVelocity(double distanceMeters) {
    return distanceMeters / timeOfFlightMap.get(distanceMeters);
  }
}
