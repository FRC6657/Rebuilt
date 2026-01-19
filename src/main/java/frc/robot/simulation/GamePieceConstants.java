// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class GamePieceConstants {

  // Game piece physical properties
  public static final double PIECE_DIAMETER_METERS = Units.inchesToMeters(5.91);
  public static final double PIECE_RADIUS_METERS = PIECE_DIAMETER_METERS / 2.0;

  // Physics constants
  public static final double GRAVITY_MPS2 = 9.81;
  public static final double COEFFICIENT_OF_RESTITUTION = 0.3; // Bounciness 
  public static final double FLOOR_FRICTION_COEFFICIENT = 0.3; // Rolling/sliding friction
  public static final double FLOOR_HEIGHT_METERS = PIECE_RADIUS_METERS;
  public static final double VELOCITY_THRESHOLD = 0.05; // Stop pieces below this velocity

  // Field Boundaries
  public static final double FIELD_LENGTH_METERS = 16.541;
  public static final double FIELD_WIDTH_METERS = 8.069;
  public static final double FIELD_MIN_X = 0.0;
  public static final double FIELD_MAX_X = FIELD_LENGTH_METERS;
  public static final double FIELD_MIN_Y = 0.0;
  public static final double FIELD_MAX_Y = FIELD_WIDTH_METERS;

  public static final Translation3d RED_TOWER_CENTER = new Translation3d(11.9, 4.03, 0.0);
  public static final Translation3d BLUE_TOWER_CENTER = new Translation3d(4.64, 4.03, 0.0);

  // Tower dimensions
  public static final double TOWER_BASE_RADIUS_METERS = 0.6; // ~24 inches
  public static final double TOWER_FUNNEL_TOP_RADIUS_METERS =
      0.55 + Units.inchesToMeters(1);
  public static final double TOWER_FUNNEL_BOTTOM_RADIUS_METERS =
      0.3 + Units.inchesToMeters(1);
  public static final double TOWER_FUNNEL_TOP_HEIGHT_METERS = 1.8;
  public static final double TOWER_FUNNEL_BOTTOM_HEIGHT_METERS =
      TOWER_FUNNEL_TOP_HEIGHT_METERS - Units.feetToMeters(1);

  // Scoring chute radius (matches funnel bottom opening)
  public static final double CHUTE_RADIUS_METERS = TOWER_FUNNEL_BOTTOM_RADIUS_METERS;

  // Hexagonal funnel geometry (6 planes)
  public static final int FUNNEL_SIDES = 6;

  /** Gets the 6 bottom vertices of the hexagonal funnel for a given tower center. */
  public static Translation3d[] getFunnelBottomVertices(Translation3d towerCenter) {
    Translation3d[] vertices = new Translation3d[FUNNEL_SIDES];
    for (int i = 0; i < FUNNEL_SIDES; i++) {
      double angle = 2 * Math.PI * i / FUNNEL_SIDES;
      vertices[i] =
          new Translation3d(
              towerCenter.getX() + TOWER_FUNNEL_BOTTOM_RADIUS_METERS * Math.cos(angle),
              towerCenter.getY() + TOWER_FUNNEL_BOTTOM_RADIUS_METERS * Math.sin(angle),
              TOWER_FUNNEL_BOTTOM_HEIGHT_METERS);
    }
    return vertices;
  }

  /** Gets the 6 top vertices of the hexagonal funnel for a given tower center. */
  public static Translation3d[] getFunnelTopVertices(Translation3d towerCenter) {
    Translation3d[] vertices = new Translation3d[FUNNEL_SIDES];
    for (int i = 0; i < FUNNEL_SIDES; i++) {
      double angle = 2 * Math.PI * i / FUNNEL_SIDES;
      vertices[i] =
          new Translation3d(
              towerCenter.getX() + TOWER_FUNNEL_TOP_RADIUS_METERS * Math.cos(angle),
              towerCenter.getY() + TOWER_FUNNEL_TOP_RADIUS_METERS * Math.sin(angle),
              TOWER_FUNNEL_TOP_HEIGHT_METERS);
    }
    return vertices;
  }

  // Simulation limits
  public static final double DEFAULT_LIFETIME_SECONDS = 5.0;
  public static final int MAX_ACTIVE_PIECES = 200;

  private GamePieceConstants() {}
}
