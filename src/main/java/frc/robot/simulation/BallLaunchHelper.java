// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.Random;

/**
 * Helper class for calculating ball launch characteristics for physics simulation based on shooter
 * RPM, hood angle, and turret angle.
 */
public final class BallLaunchHelper {

  private static final double SHOOTER_WHEEL_RADIUS_M = Units.inchesToMeters(2.0);
  private static final double PROJECTILE_MASS_KG = 0.219;
  private static final double TOTAL_MOI_KG_M2 = 0.002663;

  private static final Translation3d TURRET_CENTER =
      new Translation3d(0.118317, -0.118317, 0.511175);
  private static final Translation3d HOOD_PIVOT_OFFSET =
      new Translation3d(0, Units.inchesToMeters(-4.156585), Units.inchesToMeters(2.75));

  private static final double BALL_EXIT_RADIUS = Units.inchesToMeters(4.5);

  // Gaussian noise for spawn variation
  private static final double SPEED_NOISE_STD_DEV = 0.02; // 2% speed variation
  private static final double HEADING_NOISE_STD_DEV_RAD = 0.03; // ~1.7 degrees angular spread
  private static final Random random = new Random();

  private BallLaunchHelper() {}

  /**
   * Calculates the speed transfer percentage using recalc formula. T = (20 * J_t) / (7 * m_p *
   * (r_w/2)² + 40 * J_t)
   *
   * @return Transfer percentage (0 to 1)
   */
  private static double calculateTransferPercentage() {
    double halfRadius = SHOOTER_WHEEL_RADIUS_M / 2.0;
    double numerator = 20.0 * TOTAL_MOI_KG_M2;
    double denominator =
        7.0 * PROJECTILE_MASS_KG * halfRadius * halfRadius + 40.0 * TOTAL_MOI_KG_M2;
    return numerator / denominator;
  }

  /**
   * Calculates exit speed in feet per second from RPM using recalc formula. V_p = V_w * T where V_w
   * = RPM * r_w * 2π/60
   *
   * @param shooterRpm The shooter wheel RPM
   * @return Exit speed in feet per second
   */
  public static double rpmToSpeedFps(double shooterRpm) {
    // Convert RPM to surface speed in m/s: V_w = RPM * 2π/60 * radius
    double surfaceSpeedMps = shooterRpm * 2.0 * Math.PI / 60.0 * SHOOTER_WHEEL_RADIUS_M;

    // Apply transfer percentage
    double exitSpeedMps = surfaceSpeedMps * calculateTransferPercentage();

    return Units.metersToFeet(exitSpeedMps);
  }

  /**
   * Calculates required RPM from target exit speed in fps using inverse of recalc formula.
   *
   * @param targetSpeedFps Target exit speed in feet per second
   * @return Required shooter RPM
   */
  public static double speedFpsToRpm(double targetSpeedFps) {
    double targetSpeedMps = Units.feetToMeters(targetSpeedFps);
    double surfaceSpeedMps = targetSpeedMps / calculateTransferPercentage();
    return surfaceSpeedMps * 60.0 / (2.0 * Math.PI * SHOOTER_WHEEL_RADIUS_M);
  }

  /**
   * Calculates the elevation angle from hood angle.
   *
   * @param hoodAngleDegrees The hood angle in degrees
   * @return The elevation angle in degrees (0 = horizontal, 90 = straight up)
   */
  public static double calculateElevationAngle(double hoodAngleDegrees) {
    return 90 - hoodAngleDegrees;
  }

  /**
   * Calculates the spawn position in field coordinates based on hood and turret angles. The ball
   * exit point rotates around the hood pivot as the hood angle changes.
   *
   * @param robotPose The robot's pose on the field
   * @param hoodAngleRad The hood angle in radians
   * @param turretAngleRad The turret angle in radians
   * @return The spawn position in field coordinates
   */
  public static Translation3d calculateSpawnPosition(
      Pose2d robotPose, double hoodAngleRad, double turretAngleRad) {

    // Calculate hood pivot position in robot frame
    // First rotate hood offset by turret angle (around Z axis)
    Rotation3d turretRotation = new Rotation3d(0, 0, turretAngleRad);
    Translation3d hoodPivotRobotFrame =
        TURRET_CENTER.plus(HOOD_PIVOT_OFFSET.rotateBy(turretRotation));

    // Ball exit is at hood angle, converted from hood reference (90 - hoodAngle = elevation)
    // Add 90 deg offset for hood frame alignment
    double elevationRad = Math.toRadians(90.0) - hoodAngleRad;
    double positionAngle = elevationRad + Math.PI / 2;

    // Exit offset in hood's local frame (Y = forward in turret frame, Z = up)
    double exitOffsetY = -BALL_EXIT_RADIUS * Math.cos(positionAngle);
    double exitOffsetZ = BALL_EXIT_RADIUS * Math.sin(positionAngle);
    Translation3d exitOffsetLocal = new Translation3d(0, exitOffsetY, exitOffsetZ);

    // Rotate exit offset by turret angle
    Translation3d exitOffsetRotated = exitOffsetLocal.rotateBy(turretRotation);

    // Ball position in robot frame
    Translation3d ballPosRobotFrame = hoodPivotRobotFrame.plus(exitOffsetRotated);

    // Transform to field coordinates
    Rotation3d robotRotation = new Rotation3d(0, 0, robotPose.getRotation().getRadians());
    Translation3d rotatedPos = ballPosRobotFrame.rotateBy(robotRotation);

    return new Translation3d(
        robotPose.getX() + rotatedPos.getX(),
        robotPose.getY() + rotatedPos.getY(),
        ballPosRobotFrame.getZ());
  }

  /**
   * Calculates the launch direction in field coordinates.
   *
   * @param turretAngleDegrees The turret angle in degrees (robot-relative)
   * @param robotPose The robot's pose on the field
   * @return The launch direction as a Rotation2d in field coordinates
   */
  public static Rotation2d calculateLaunchDirection(double turretAngleDegrees, Pose2d robotPose) {
    return robotPose.getRotation().plus(Rotation2d.fromDegrees(turretAngleDegrees - 90));
  }

  /**
   * Calculates the turret's velocity in field coordinates due to robot motion. Accounts for both
   * linear velocity and tangential velocity from rotation.
   *
   * @param chassisSpeeds The robot's chassis speeds (robot-relative)
   * @param robotPose The robot's pose on the field
   * @return The turret velocity in field coordinates (m/s)
   */
  public static Translation2d calculateTurretVelocity(ChassisSpeeds chassisSpeeds, Pose2d robotPose) {
    // Turret offset from robot center (robot-relative)
    double turretOffsetX = TURRET_CENTER.getX();
    double turretOffsetY = TURRET_CENTER.getY();

    // Tangential velocity from robot rotation at turret position
    // v_tangential = ω × r
    // v_tangential_x = -ω * r_y
    // v_tangential_y = ω * r_x
    double omega = chassisSpeeds.omegaRadiansPerSecond;
    double tangentialVx = -omega * turretOffsetY;
    double tangentialVy = omega * turretOffsetX;

    // Total velocity in robot frame = linear + tangential
    double robotVx = chassisSpeeds.vxMetersPerSecond + tangentialVx;
    double robotVy = chassisSpeeds.vyMetersPerSecond + tangentialVy;

    // Transform to field coordinates
    Translation2d robotRelativeVel = new Translation2d(robotVx, robotVy);
    return robotRelativeVel.rotateBy(robotPose.getRotation());
  }

  /**
   * Spawns a game piece using the calculated launch characteristics. Does not account for robot
   * motion.
   *
   * @param simulation The game piece simulation instance
   * @param shooterRpm The shooter wheel RPM
   * @param hoodAngleDegrees The hood angle in degrees
   * @param turretAngleDegrees The turret angle in degrees (robot-relative)
   * @param robotPose The robot's pose on the field
   */
  public static void spawnWithLaunchCharacteristics(
      GamePieceSimulation simulation,
      double shooterRpm,
      double hoodAngleDegrees,
      double turretAngleDegrees,
      Pose2d robotPose) {
    spawnWithLaunchCharacteristics(
        simulation,
        shooterRpm,
        hoodAngleDegrees,
        turretAngleDegrees,
        robotPose,
        new ChassisSpeeds());
  }

  /**
   * Spawns a game piece using the calculated launch characteristics, accounting for robot motion.
   * The piece inherits the turret's velocity from the moving robot.
   *
   * @param simulation The game piece simulation instance
   * @param shooterRpm The shooter wheel RPM
   * @param hoodAngleDegrees The hood angle in degrees
   * @param turretAngleDegrees The turret angle in degrees (robot-relative)
   * @param robotPose The robot's pose on the field
   * @param chassisSpeeds The robot's chassis speeds (robot-relative)
   */
  public static void spawnWithLaunchCharacteristics(
      GamePieceSimulation simulation,
      double shooterRpm,
      double hoodAngleDegrees,
      double turretAngleDegrees,
      Pose2d robotPose,
      ChassisSpeeds chassisSpeeds) {

    // Convert angles to radians
    double hoodAngleRad = Math.toRadians(hoodAngleDegrees - 10.0); // Offset for model position
    double turretAngleRad = Math.toRadians(turretAngleDegrees);

    // Calculate spawn position based on hood and turret angles
    Translation3d spawnPosition = calculateSpawnPosition(robotPose, hoodAngleRad, turretAngleRad);

    Rotation2d launchDirection = calculateLaunchDirection(turretAngleDegrees, robotPose);
    double elevationDegrees = calculateElevationAngle(hoodAngleDegrees);
    double speedMps = Units.feetToMeters(rpmToSpeedFps(shooterRpm));

    // Apply Gaussian noise to heading (additive angular offset)
    double headingNoiseRad = random.nextGaussian() * HEADING_NOISE_STD_DEV_RAD;
    Rotation2d noisyDirection = launchDirection.plus(Rotation2d.fromRadians(headingNoiseRad));

    // Apply Gaussian noise to speed (multiplicative)
    double speedNoise = random.nextGaussian() * SPEED_NOISE_STD_DEV;
    double noisySpeedMps = speedMps * (1.0 + speedNoise);

    // Calculate launch velocity from shooter with noise applied
    double elevationRad = Math.toRadians(elevationDegrees);
    double horizontalSpeed = noisySpeedMps * Math.cos(elevationRad);
    double verticalSpeed = noisySpeedMps * Math.sin(elevationRad);

    double launchVx = horizontalSpeed * noisyDirection.getCos();
    double launchVy = horizontalSpeed * noisyDirection.getSin();
    double launchVz = verticalSpeed;

    // Add turret velocity from robot motion
    Translation2d turretVelocity = calculateTurretVelocity(chassisSpeeds, robotPose);

    Translation3d totalVelocity =
        new Translation3d(
            launchVx + turretVelocity.getX(), launchVy + turretVelocity.getY(), launchVz);

    simulation.spawnPieceWithVelocity(spawnPosition, totalVelocity);
  }
}
