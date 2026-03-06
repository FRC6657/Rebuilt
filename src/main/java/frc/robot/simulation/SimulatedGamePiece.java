// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SimulatedGamePiece {

  private Translation3d position;
  private Translation3d velocity;
  private double lifetime;
  private final double maxLifetime;
  private boolean onGround;
  private boolean scored;

  public SimulatedGamePiece(Translation3d position, Translation3d velocity, double maxLifetime) {
    this.position = position;
    this.velocity = velocity;
    this.lifetime = 0.0;
    this.maxLifetime = maxLifetime;
    this.onGround = false;
    this.scored = false;
  }

  public SimulatedGamePiece(Translation3d position, Translation3d velocity) {
    this(position, velocity, GamePieceConstants.DEFAULT_LIFETIME_SECONDS);
  }

  public void update(double dtSeconds) {
    lifetime += dtSeconds;

    // Apply gravity to velocity
    velocity =
        new Translation3d(
            velocity.getX(),
            velocity.getY(),
            velocity.getZ() - GamePieceConstants.GRAVITY_MPS2 * dtSeconds);

    // Calculate intended new position
    Translation3d newPosition =
        new Translation3d(
            position.getX() + velocity.getX() * dtSeconds,
            position.getY() + velocity.getY() * dtSeconds,
            position.getZ() + velocity.getZ() * dtSeconds);

    // Pre-check tower collisions and adjust movement if needed
    newPosition = preCheckTowerCollisions(position, newPosition);

    // Apply the (possibly adjusted) new position
    position = newPosition;

    // Handle remaining collisions
    handleTowerCollisions();
    handleChuteCollisions();
    handleFloorCollision(dtSeconds);
    handleWallCollisions();
  }

  /** Handles collision with the scoring chute */
  private void handleChuteCollisions() {
    checkSingleChuteCollision(GamePieceConstants.RED_TOWER_CENTER);
    checkSingleChuteCollision(GamePieceConstants.BLUE_TOWER_CENTER);
  }

  private void checkSingleChuteCollision(Translation3d towerCenter) {
    double chuteRadius = GamePieceConstants.CHUTE_RADIUS_METERS;
    double funnelBottom = GamePieceConstants.TOWER_FUNNEL_BOTTOM_HEIGHT_METERS;

    double dx = position.getX() - towerCenter.getX();
    double dy = position.getY() - towerCenter.getY();
    double horizontalDist = Math.sqrt(dx * dx + dy * dy);

    // Despawn if inside chute hole and below funnel bottom
    if (horizontalDist < chuteRadius && position.getZ() < funnelBottom) {
      scored = true;
    }
  }

  /** Pre-checks tower collisions and returns adjusted position to prevent clipping. */
  private Translation3d preCheckTowerCollisions(Translation3d oldPos, Translation3d newPos) {
    newPos = preCheckSingleTower(oldPos, newPos, GamePieceConstants.RED_TOWER_CENTER);
    newPos = preCheckSingleTower(oldPos, newPos, GamePieceConstants.BLUE_TOWER_CENTER);
    return newPos;
  }

  private Translation3d preCheckSingleTower(
      Translation3d oldPos, Translation3d newPos, Translation3d towerCenter) {
    double pieceRadius = GamePieceConstants.PIECE_RADIUS_METERS;
    double funnelBottom = GamePieceConstants.TOWER_FUNNEL_BOTTOM_HEIGHT_METERS;
    double funnelTop = GamePieceConstants.TOWER_FUNNEL_TOP_HEIGHT_METERS;
    double radiusTop = GamePieceConstants.TOWER_FUNNEL_TOP_RADIUS_METERS;

    // Quick bounds check
    double dx = newPos.getX() - towerCenter.getX();
    double dy = newPos.getY() - towerCenter.getY();
    double horizontalDist = Math.sqrt(dx * dx + dy * dy);

    if (newPos.getZ() < funnelBottom - pieceRadius
        || newPos.getZ() > funnelTop + pieceRadius
        || horizontalDist > radiusTop + pieceRadius * 2) {
      return newPos;
    }

    Translation3d[] bottomVerts = GamePieceConstants.getFunnelBottomVertices(towerCenter);
    Translation3d[] topVerts = GamePieceConstants.getFunnelTopVertices(towerCenter);

    // Check each plane for intersection with movement ray
    for (int i = 0; i < GamePieceConstants.FUNNEL_SIDES; i++) {
      int next = (i + 1) % GamePieceConstants.FUNNEL_SIDES;

      Translation3d bl = bottomVerts[i];
      Translation3d br = bottomVerts[next];
      Translation3d tl = topVerts[i];

      // Calculate plane normal
      double e1x = br.getX() - bl.getX();
      double e1y = br.getY() - bl.getY();
      double e1z = br.getZ() - bl.getZ();

      double e2x = tl.getX() - bl.getX();
      double e2y = tl.getY() - bl.getY();
      double e2z = tl.getZ() - bl.getZ();

      double nx = e1y * e2z - e1z * e2y;
      double ny = e1z * e2x - e1x * e2z;
      double nz = e1x * e2y - e1y * e2x;

      double nLen = Math.sqrt(nx * nx + ny * ny + nz * nz);
      if (nLen < 0.0001) continue;
      nx /= nLen;
      ny /= nLen;
      nz /= nLen;

      // Ensure normal points inward
      double toCenterX = towerCenter.getX() - bl.getX();
      double toCenterY = towerCenter.getY() - bl.getY();
      if (nx * toCenterX + ny * toCenterY < 0) {
        nx = -nx;
        ny = -ny;
        nz = -nz;
      }

      // Check if new position penetrates plane
      double px = newPos.getX() - bl.getX();
      double py = newPos.getY() - bl.getY();
      double pz = newPos.getZ() - bl.getZ();
      double distNew = px * nx + py * ny + pz * nz;

      if (distNew < pieceRadius) {
        // Would penetrate - check if we're crossing from outside
        double ox = oldPos.getX() - bl.getX();
        double oy = oldPos.getY() - bl.getY();
        double oz = oldPos.getZ() - bl.getZ();
        double distOld = ox * nx + oy * ny + oz * nz;

        if (distOld >= pieceRadius) {
          // Crossing from outside to inside - find intersection and stop there
          double t = (distOld - pieceRadius) / (distOld - distNew);
          t = Math.max(0, Math.min(1, t));

          newPos =
              new Translation3d(
                  oldPos.getX() + t * (newPos.getX() - oldPos.getX()),
                  oldPos.getY() + t * (newPos.getY() - oldPos.getY()),
                  oldPos.getZ() + t * (newPos.getZ() - oldPos.getZ()));

          // Reflect velocity off this plane
          double vDotN = velocity.getX() * nx + velocity.getY() * ny + velocity.getZ() * nz;
          if (vDotN < 0) {
            double restitution = GamePieceConstants.COEFFICIENT_OF_RESTITUTION;
            velocity =
                new Translation3d(
                    velocity.getX() - (1 + restitution) * vDotN * nx,
                    velocity.getY() - (1 + restitution) * vDotN * ny,
                    velocity.getZ() - (1 + restitution) * vDotN * nz);
          }
        }
      }
    }

    return newPos;
  }

  private void handleFloorCollision(double dtSeconds) {
    if (position.getZ() < GamePieceConstants.FLOOR_HEIGHT_METERS) {
      // Clamp to floor
      position =
          new Translation3d(
              position.getX(), position.getY(), GamePieceConstants.FLOOR_HEIGHT_METERS);

      // Bounce with coefficient of restitution
      if (velocity.getZ() < 0) {
        double newVz = -velocity.getZ() * GamePieceConstants.COEFFICIENT_OF_RESTITUTION;

        // If bounce is too small, stop vertical motion
        if (Math.abs(newVz) < GamePieceConstants.VELOCITY_THRESHOLD) {
          newVz = 0;
        }

        velocity = new Translation3d(velocity.getX(), velocity.getY(), newVz);
      }

      onGround = true;
    } else {
      onGround = false;
    }

    // Apply floor friction when on ground
    if (onGround) {
      applyFloorFriction(dtSeconds);
    }
  }

  private void applyFloorFriction(double dtSeconds) {
    double vx = velocity.getX();
    double vy = velocity.getY();

    double horizontalSpeed = Math.sqrt(vx * vx + vy * vy);

    if (horizontalSpeed > GamePieceConstants.VELOCITY_THRESHOLD) {
      // Apply friction deceleration
      double frictionDecel =
          GamePieceConstants.FLOOR_FRICTION_COEFFICIENT
              * GamePieceConstants.GRAVITY_MPS2
              * dtSeconds;

      double newSpeed = Math.max(0, horizontalSpeed - frictionDecel);
      double scale = newSpeed / horizontalSpeed;

      velocity = new Translation3d(vx * scale, vy * scale, velocity.getZ());
    } else {
      // Stop completely if below threshold
      velocity = new Translation3d(0, 0, velocity.getZ());
    }
  }

  private void handleWallCollisions() {
    double newX = position.getX();
    double newY = position.getY();
    double newVx = velocity.getX();
    double newVy = velocity.getY();

    double radius = GamePieceConstants.PIECE_RADIUS_METERS;

    // X-axis walls
    if (position.getX() - radius < GamePieceConstants.FIELD_MIN_X) {
      newX = GamePieceConstants.FIELD_MIN_X + radius;
      newVx = 0;
    } else if (position.getX() + radius > GamePieceConstants.FIELD_MAX_X) {
      newX = GamePieceConstants.FIELD_MAX_X - radius;
      newVx = 0;
    }

    // Y-axis walls
    if (position.getY() - radius < GamePieceConstants.FIELD_MIN_Y) {
      newY = GamePieceConstants.FIELD_MIN_Y + radius;
      newVy = 0;
    } else if (position.getY() + radius > GamePieceConstants.FIELD_MAX_Y) {
      newY = GamePieceConstants.FIELD_MAX_Y - radius;
      newVy = 0;
    }

    position = new Translation3d(newX, newY, position.getZ());
    velocity = new Translation3d(newVx, newVy, velocity.getZ());
  }

  private void handleTowerCollisions() {
    // Check collision with both alliance towers
    handleSingleTowerCollision(GamePieceConstants.RED_TOWER_CENTER);
    handleSingleTowerCollision(GamePieceConstants.BLUE_TOWER_CENTER);
  }

  private void handleSingleTowerCollision(Translation3d towerCenter) {
    double radius = GamePieceConstants.PIECE_RADIUS_METERS;

    // Calculate horizontal distance from tower center
    double dx = position.getX() - towerCenter.getX();
    double dy = position.getY() - towerCenter.getY();
    double horizontalDist = Math.sqrt(dx * dx + dy * dy);
    double height = position.getZ();

    double funnelBottomRadius = GamePieceConstants.TOWER_FUNNEL_BOTTOM_RADIUS_METERS;
    double funnelBottomHeight = GamePieceConstants.TOWER_FUNNEL_BOTTOM_HEIGHT_METERS;

    // Check if piece is inside the funnel (within funnel bottom radius)
    boolean insideFunnel = horizontalDist < funnelBottomRadius - radius;

    // Check collision with tower base (cylinder from floor to funnel bottom)
    // Only apply to pieces OUTSIDE the funnel
    if (!insideFunnel && height < funnelBottomHeight + radius) {
      double baseCollisionDist = GamePieceConstants.TOWER_BASE_RADIUS_METERS + radius;
      if (horizontalDist < baseCollisionDist) {
        // Push piece out radially
        double pushDist = baseCollisionDist - horizontalDist;
        double angle = Math.atan2(dy, dx);

        position =
            new Translation3d(
                position.getX() + pushDist * Math.cos(angle),
                position.getY() + pushDist * Math.sin(angle),
                position.getZ());

        // Stop radial velocity (no bounce off base)
        double vRadial = velocity.getX() * Math.cos(angle) + velocity.getY() * Math.sin(angle);
        if (vRadial < 0) {
          velocity =
              new Translation3d(
                  velocity.getX() - vRadial * Math.cos(angle),
                  velocity.getY() - vRadial * Math.sin(angle),
                  velocity.getZ());
        }
      }
    }

    // Backup plane collision to catch pieces that slip through pre-check
    handleFunnelPlaneCollisions(towerCenter);
  }

  private void handleFunnelPlaneCollisions(Translation3d towerCenter) {
    double pieceRadius = GamePieceConstants.PIECE_RADIUS_METERS;
    double height = position.getZ();

    double funnelBottom = GamePieceConstants.TOWER_FUNNEL_BOTTOM_HEIGHT_METERS;
    double funnelTop = GamePieceConstants.TOWER_FUNNEL_TOP_HEIGHT_METERS;
    double radiusTop = GamePieceConstants.TOWER_FUNNEL_TOP_RADIUS_METERS;

    // Only check if in funnel height range
    if (height < funnelBottom - pieceRadius || height > funnelTop + pieceRadius) {
      return;
    }

    double dx = position.getX() - towerCenter.getX();
    double dy = position.getY() - towerCenter.getY();
    double horizontalDist = Math.sqrt(dx * dx + dy * dy);

    // Must be near funnel
    if (horizontalDist > radiusTop + pieceRadius * 2) {
      return;
    }

    Translation3d[] bottomVerts = GamePieceConstants.getFunnelBottomVertices(towerCenter);
    Translation3d[] topVerts = GamePieceConstants.getFunnelTopVertices(towerCenter);

    // Accumulate total push and combined normal for all penetrating planes
    double totalPushX = 0, totalPushY = 0, totalPushZ = 0;
    double combinedNx = 0, combinedNy = 0, combinedNz = 0;
    int penetratingPlanes = 0;

    for (int i = 0; i < GamePieceConstants.FUNNEL_SIDES; i++) {
      int next = (i + 1) % GamePieceConstants.FUNNEL_SIDES;

      Translation3d bl = bottomVerts[i];
      Translation3d br = bottomVerts[next];
      Translation3d tl = topVerts[i];

      // Calculate plane normal
      double e1x = br.getX() - bl.getX();
      double e1y = br.getY() - bl.getY();
      double e1z = br.getZ() - bl.getZ();

      double e2x = tl.getX() - bl.getX();
      double e2y = tl.getY() - bl.getY();
      double e2z = tl.getZ() - bl.getZ();

      double nx = e1y * e2z - e1z * e2y;
      double ny = e1z * e2x - e1x * e2z;
      double nz = e1x * e2y - e1y * e2x;

      double nLen = Math.sqrt(nx * nx + ny * ny + nz * nz);
      if (nLen < 0.0001) continue;
      nx /= nLen;
      ny /= nLen;
      nz /= nLen;

      // Ensure normal points inward
      double toCenterX = towerCenter.getX() - bl.getX();
      double toCenterY = towerCenter.getY() - bl.getY();
      if (nx * toCenterX + ny * toCenterY < 0) {
        nx = -nx;
        ny = -ny;
        nz = -nz;
      }

      // Distance from piece center to plane
      double px = position.getX() - bl.getX();
      double py = position.getY() - bl.getY();
      double pz = position.getZ() - bl.getZ();
      double dist = px * nx + py * ny + pz * nz;

      double penetration = pieceRadius - dist;
      if (penetration > 0) {
        // Accumulate push for this plane
        totalPushX += nx * penetration;
        totalPushY += ny * penetration;
        totalPushZ += nz * penetration;

        // Accumulate normal (weighted by penetration)
        combinedNx += nx * penetration;
        combinedNy += ny * penetration;
        combinedNz += nz * penetration;
        penetratingPlanes++;
      }
    }

    // Apply accumulated collision response
    if (penetratingPlanes > 0) {
      // Push piece out
      position =
          new Translation3d(
              position.getX() + totalPushX,
              position.getY() + totalPushY,
              position.getZ() + totalPushZ);

      // Normalize combined normal
      double nLen =
          Math.sqrt(combinedNx * combinedNx + combinedNy * combinedNy + combinedNz * combinedNz);
      if (nLen > 0.0001) {
        combinedNx /= nLen;
        combinedNy /= nLen;
        combinedNz /= nLen;

        // Reflect velocity off combined normal
        double vDotN =
            velocity.getX() * combinedNx
                + velocity.getY() * combinedNy
                + velocity.getZ() * combinedNz;
        if (vDotN < 0) {
          double restitution = GamePieceConstants.COEFFICIENT_OF_RESTITUTION;
          velocity =
              new Translation3d(
                  velocity.getX() - (1 + restitution) * vDotN * combinedNx,
                  velocity.getY() - (1 + restitution) * vDotN * combinedNy,
                  velocity.getZ() - (1 + restitution) * vDotN * combinedNz);
        }
      }
    }
  }

  public boolean isExpired() {
    return lifetime >= maxLifetime;
  }

  public boolean isScored() {
    return scored;
  }

  public Translation3d getPosition() {
    return position;
  }

  public void setPosition(Translation3d position) {
    this.position = position;
  }

  public Translation3d getVelocity() {
    return velocity;
  }

  public void setVelocity(Translation3d velocity) {
    this.velocity = velocity;
  }

  public Pose3d getPose() {
    return new Pose3d(position, new Rotation3d());
  }

  public double getDistanceTo(Translation3d other) {
    return position.getDistance(other);
  }
}
