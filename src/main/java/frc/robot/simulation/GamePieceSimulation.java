package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class GamePieceSimulation {

  private static GamePieceSimulation instance;

  private final List<SimulatedGamePiece> pieces = new ArrayList<>();
  private static final double DT_SECONDS = 0.02;

  private int totalScored = 0;

  private GamePieceSimulation() {
    logTowerGeometry();
  }

  public static GamePieceSimulation getInstance() {
    if (instance == null) {
      instance = new GamePieceSimulation();
    }
    return instance;
  }

  private void logTowerGeometry() {
    logSingleTowerGeometry(GamePieceConstants.RED_TOWER_CENTER, "Red");
    logSingleTowerGeometry(GamePieceConstants.BLUE_TOWER_CENTER, "Blue");
  }

  private void logSingleTowerGeometry(Translation3d towerCenter, String allianceName) {
    Translation3d[] bottomVerts = GamePieceConstants.getFunnelBottomVertices(towerCenter);
    Translation3d[] topVerts = GamePieceConstants.getFunnelTopVertices(towerCenter);

    // Log wireframe for visualization
    Pose3d[] wireframe = new Pose3d[GamePieceConstants.FUNNEL_SIDES * 4];
    for (int i = 0; i < GamePieceConstants.FUNNEL_SIDES; i++) {
      int next = (i + 1) % GamePieceConstants.FUNNEL_SIDES;
      wireframe[i * 4] = new Pose3d(bottomVerts[i], new Rotation3d());
      wireframe[i * 4 + 1] = new Pose3d(topVerts[i], new Rotation3d());
      wireframe[i * 4 + 2] = new Pose3d(topVerts[next], new Rotation3d());
      wireframe[i * 4 + 3] = new Pose3d(bottomVerts[next], new Rotation3d());
    }
    Logger.recordOutput("GamePieceSim/" + allianceName + "Tower/Wireframe", wireframe);
  }

  /**
   * Spawns a game piece with a direct velocity vector.
   *
   * @param position The starting position in meters
   * @param velocityMps The velocity vector in meters per second
   */
  public void spawnPieceWithVelocity(Translation3d position, Translation3d velocityMps) {
    if (pieces.size() >= GamePieceConstants.MAX_ACTIVE_PIECES) {
      pieces.remove(0);
    }

    pieces.add(new SimulatedGamePiece(position, velocityMps));
  }

  /**
   * Removes all pieces within the specified radius of a center point.
   *
   * @param center The center point to check
   * @param radiusMeters The radius in meters
   */
  public void removePiecesNear(Translation3d center, double radiusMeters) {
    pieces.removeIf(piece -> piece.getDistanceTo(center) <= radiusMeters);
  }

  /**
   * Checks if any piece exists within the specified radius of a center point.
   *
   * @param center The center point to check
   * @param radiusMeters The radius in meters
   * @return true if at least one piece is within the radius
   */
  public boolean hasPieceNear(Translation3d center, double radiusMeters) {
    for (SimulatedGamePiece piece : pieces) {
      if (piece.getDistanceTo(center) <= radiusMeters) {
        return true;
      }
    }
    return false;
  }

  /** Removes all active game pieces. */
  public void clearAll() {
    pieces.clear();
  }

  /** Returns the current number of active pieces. */
  public int getPieceCount() {
    return pieces.size();
  }

  /** Returns the total number of scored pieces this session. */
  public int getTotalScored() {
    return totalScored;
  }

  /** Resets the score counter. */
  public void resetScore() {
    totalScored = 0;
  }

  /** Handles collisions between all pairs of game pieces. */
  private void handlePieceToPieceCollisions() {
    double diameter = GamePieceConstants.PIECE_DIAMETER_METERS;
    double restitution = GamePieceConstants.COEFFICIENT_OF_RESTITUTION;

    // Check all pairs of pieces
    for (int i = 0; i < pieces.size(); i++) {
      for (int j = i + 1; j < pieces.size(); j++) {
        SimulatedGamePiece pieceA = pieces.get(i);
        SimulatedGamePiece pieceB = pieces.get(j);

        Translation3d posA = pieceA.getPosition();
        Translation3d posB = pieceB.getPosition();

        // Calculate distance between centers
        double dx = posB.getX() - posA.getX();
        double dy = posB.getY() - posA.getY();
        double dz = posB.getZ() - posA.getZ();
        double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

        // Check for collision (spheres overlap)
        if (dist < diameter && dist > 0.001) {
          // Normalize collision vector
          double nx = dx / dist;
          double ny = dy / dist;
          double nz = dz / dist;

          // Separate the pieces (push apart equally)
          double overlap = diameter - dist;
          double separationDist = overlap / 2.0 + 0.001;

          pieceA.setPosition(
              new Translation3d(
                  posA.getX() - nx * separationDist,
                  posA.getY() - ny * separationDist,
                  posA.getZ() - nz * separationDist));

          pieceB.setPosition(
              new Translation3d(
                  posB.getX() + nx * separationDist,
                  posB.getY() + ny * separationDist,
                  posB.getZ() + nz * separationDist));

          // Get velocities
          Translation3d velA = pieceA.getVelocity();
          Translation3d velB = pieceB.getVelocity();

          // Calculate relative velocity along collision normal
          double dvx = velA.getX() - velB.getX();
          double dvy = velA.getY() - velB.getY();
          double dvz = velA.getZ() - velB.getZ();
          double relVelNormal = dvx * nx + dvy * ny + dvz * nz;

          // Only resolve if pieces are approaching each other
          if (relVelNormal > 0) {
            // For equal mass elastic collision, pieces exchange velocity components
            // along the collision normal. With restitution, we scale the impulse.
            double impulse = relVelNormal * (1 + restitution) / 2.0;

            pieceA.setVelocity(
                new Translation3d(
                    velA.getX() - impulse * nx,
                    velA.getY() - impulse * ny,
                    velA.getZ() - impulse * nz));

            pieceB.setVelocity(
                new Translation3d(
                    velB.getX() + impulse * nx,
                    velB.getY() + impulse * ny,
                    velB.getZ() + impulse * nz));
          }
        }
      }
    }
  }

  /** Updates all game pieces and logs to AdvantageScope. Call this from robotPeriodic(). */
  public void update() {
    // Update physics for all pieces
    for (SimulatedGamePiece piece : pieces) {
      piece.update(DT_SECONDS);
    }

    // Handle piece-to-piece collisions
    handlePieceToPieceCollisions();

    // Remove expired or scored pieces
    Iterator<SimulatedGamePiece> iterator = pieces.iterator();
    while (iterator.hasNext()) {
      SimulatedGamePiece piece = iterator.next();
      if (piece.isExpired()) {
        iterator.remove();
      } else if (piece.isScored()) {
        totalScored++;
        iterator.remove();
      }
    }

    // Log pieces to AdvantageScope
    Pose3d[] poses = new Pose3d[pieces.size()];
    for (int i = 0; i < pieces.size(); i++) {
      poses[i] = pieces.get(i).getPose();
    }
    Logger.recordOutput("GamePieceSim/Pieces", poses);

    // Re-log tower geometry periodically (in case AdvantageScope reconnects)
    logTowerGeometry();
  }
}
