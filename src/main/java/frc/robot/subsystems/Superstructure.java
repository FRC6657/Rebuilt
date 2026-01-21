package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.simulation.GamePieceConstants;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  // Subsystems
  Drivebase drivebase;
  // Turret turret;
  // Hood hood;
  // Shooter shoot;

  // Fake test visualization angles (radians)
  private double turretAngle = Math.PI / 4;
  private double hoodAngle = 0.0;
  Pose2d turretPos = new Pose2d();
  Pose2d turretTarget =
      new Pose2d(
          GamePieceConstants.BLUE_TOWER_CENTER.getX(),
          GamePieceConstants.BLUE_TOWER_CENTER.getY(),
          new Rotation2d());

  public Superstructure(Drivebase drivebase) {
    this.drivebase = drivebase;
    // this.turret = turret;
    // this.hood = hood;
    // this.shoot = shoot;
  }

  @AutoLogOutput(key = "3DComponents")
  public Pose3d[] get3DComponents() {

    // Turret / Shooter
    Rotation3d turretRotation = new Rotation3d(0, 0, turretAngle);
    Translation3d rotatedHoodOffset = TurretConstants.HOOD_OFFSET.rotateBy(turretRotation);
    Translation3d hoodPosition = TurretConstants.TURRET_CENTER.plus(rotatedHoodOffset);
    Rotation3d hoodLocalPitch = new Rotation3d(hoodAngle, 0, 0);
    Rotation3d hoodRotation = hoodLocalPitch.rotateBy(turretRotation);

    return new Pose3d[] {
      new Pose3d(TurretConstants.TURRET_CENTER, turretRotation),
      new Pose3d(hoodPosition, hoodRotation)
    };
  }

  public Rotation2d findTargetAngle(double x0, double y0, double x1, double y1) {
    if (x0 < x1) {
      return new Rotation2d(
          Math.PI / 2
              + Math.atan(
                  (turretPos.getY() - turretTarget.getY())
                      / (turretPos.getX() - turretTarget.getX())));
    } else {
      return new Rotation2d(
          Math.PI / -2
              + Math.atan(
                  (turretPos.getY() - turretTarget.getY())
                      / (turretPos.getX() - turretTarget.getX())));
    }
  }

  public void changeTurretAngle(double radians) {
    turretPos =
        drivebase
            .getPose()
            .transformBy(
                new Transform2d(
                    TurretConstants.TURRET_CENTER.getX(),
                    TurretConstants.TURRET_CENTER.getY(),
                    new Rotation2d()));

    Rotation2d targetAngle =
        findTargetAngle(
            turretPos.getX(), turretPos.getY(), turretTarget.getX(), turretTarget.getY());
    
    turretAngle = targetAngle.getRadians();

    turretPos = new Pose2d(turretPos.getX(), turretPos.getY(), targetAngle);

    Logger.recordOutput(
        "turretPos",
        new Pose3d(
            turretPos.getX(),
            turretPos.getY(),
            TurretConstants.TURRET_CENTER.getZ(),
            new Rotation3d(
                Rotation2d.fromRadians(
                    drivebase.getPose().getRotation().getRadians() + turretAngle - Math.PI / 2))));
  }

  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }
}
