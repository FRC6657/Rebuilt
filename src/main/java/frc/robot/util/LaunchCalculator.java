package frc.robot.util;

// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.turret.*;
import frc.robot.util.geometry.*;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {
  private static LaunchCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (GlobalConstants.mainLoopFrequency * 0.1));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (GlobalConstants.mainLoopFrequency * 0.1));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  private Pose2d currentPose = new Pose2d();
  private ChassisSpeeds fieldSpeed = new ChassisSpeeds();

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 0.9;
    maxDistance = 5.5;
    phaseDelay = 0.08;

    // key is in meters

    launchHoodAngleMap.put(0.9, Rotation2d.fromDegrees(10));
    launchHoodAngleMap.put(1.7, Rotation2d.fromDegrees(17));
    launchHoodAngleMap.put(2.7, Rotation2d.fromDegrees(27));
    launchHoodAngleMap.put(3.7, Rotation2d.fromDegrees(30));
    launchHoodAngleMap.put(4.7, Rotation2d.fromDegrees(33));
    launchHoodAngleMap.put(5.5, Rotation2d.fromDegrees(35));

    launchFlywheelSpeedMap.put(0.9, Units.rotationsPerMinuteToRadiansPerSecond(2400));
    launchFlywheelSpeedMap.put(1.7, Units.rotationsPerMinuteToRadiansPerSecond(2500));
    launchFlywheelSpeedMap.put(2.7, Units.rotationsPerMinuteToRadiansPerSecond(2600));
    launchFlywheelSpeedMap.put(3.7, Units.rotationsPerMinuteToRadiansPerSecond(2800));
    launchFlywheelSpeedMap.put(4.7, Units.rotationsPerMinuteToRadiansPerSecond(3000));
    launchFlywheelSpeedMap.put(5.5, Units.rotationsPerMinuteToRadiansPerSecond(3200));

    timeOfFlightMap.put(0.9, 1.0);
    timeOfFlightMap.put(1.7, 1.0);
    timeOfFlightMap.put(2.7, 1.0);
    timeOfFlightMap.put(3.7, 1.06);
    timeOfFlightMap.put(4.7, 1.1);
    timeOfFlightMap.put(5.6, 1.2);
  }

  /**
   * @param target make the target be from the perspective of the Blue Alliance, it will
   *     automatically be flipped
   * @return
   */
  public LaunchingParameters getParameters(Translation2d target) {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = currentPose;
    ChassisSpeeds robotRelativeVelocity = fieldSpeed;
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    target = AllianceFlipUtil.apply(target);
    Pose2d turretPosition = estimatedPose.transformBy(TurretConstants.robotToTurret);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = fieldSpeed;
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            - robotVelocity.omegaRadiansPerSecond
                * (TurretConstants.robotToTurret.getX() * Math.sin(robotAngle)
                    + TurretConstants.robotToTurret.getY() * Math.cos(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (TurretConstants.robotToTurret.getX() * Math.cos(robotAngle)
                    - TurretConstants.robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians()
                / (1 / GlobalConstants.mainLoopFrequency));
    hoodVelocity =
        hoodAngleFilter.calculate(
            (hoodAngle - lastHoodAngle) / (1 / GlobalConstants.mainLoopFrequency));
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);
    Logger.recordOutput(
        "LaunchCalculator/TurretTarget",
        new Pose2d(target.getX(), target.getY(), new Rotation2d()));

    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public void setEstimatedPose(Pose2d pose) {
    currentPose = pose;
  }

  public void setFieldVelocity(ChassisSpeeds speeds) {
    fieldSpeed = speeds;
  }
}
