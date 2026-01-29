package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.simulation.GamePieceConstants;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  // Subsystems
  Drivebase drivebase;
  Turret turret;
  Hood hood;
  Shooter shoot;
  Floor floor;
  Intake intake;
  Tunnel tunnel;

  // Fake test visualization angles (radians)
  private double turretAngle = Math.PI / 4;
  private double hoodAngle = 0.0;
  Translation2d turretTarget =
      new Translation2d(
          GamePieceConstants.BLUE_TOWER_CENTER.getX(), GamePieceConstants.BLUE_TOWER_CENTER.getY());

  public Superstructure(
      Drivebase drivebase,
      Turret turret,
      Hood hood,
      Shooter shoot,
      Intake intake,
      Floor floor,
      Tunnel tunnel) {
    this.drivebase = drivebase;
    this.turret = turret;
    this.hood = hood;
    this.shoot = shoot;
    this.floor = floor;
    this.intake = intake;
    this.tunnel = tunnel;
  }

  @AutoLogOutput(key = "3DComponents")
  public Pose3d[] get3DComponents() {
    // double turretAngle = turret.getPosition();
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
                  (TurretConstants.TURRET_CENTER.getY() - turretTarget.getY())
                      / (TurretConstants.TURRET_CENTER.getX() - turretTarget.getX())));
    } else {
      return new Rotation2d(
          Math.PI / -2
              + Math.atan(
                  (TurretConstants.TURRET_CENTER.getY() - turretTarget.getY())
                      / (TurretConstants.TURRET_CENTER.getX() - turretTarget.getX())));
    }
  }

  public Translation2d getTurretGlobalPosition() {
    return drivebase
        .getPose()
        .getTranslation()
        .plus(
            new Translation2d(
                TurretConstants.TURRET_CENTER.getX(), TurretConstants.TURRET_CENTER.getY()));
  }

  public Rotation2d getGlobalTargetHeading(Translation2d goalPose) {
    return Rotation2d.fromRadians(
        Math.atan2(
            (goalPose.getY() - getTurretGlobalPosition().getY()),
            (goalPose.getX() - getTurretGlobalPosition().getX())));
  }

  public Rotation2d getRelativeTurretHeading(Rotation2d globalHeading) {
    return globalHeading.minus(drivebase.getPose().getRotation()).minus(Rotation2d.kCW_90deg);
  }

  public void runTurretTest() {
    Rotation2d targetAngle = getGlobalTargetHeading(turretTarget);
    turretAngle = getRelativeTurretHeading(targetAngle).getRadians();
  }

  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }

  public Command shoot() {
    return Commands.sequence(logMessage("Shoot"), shoot.changeSetpoint(12));
  }

  public Command HomeRobot() {
    return Commands.sequence(
        logMessage("Home Robot"),
        shoot.changeSetpoint(0),
        tunnel.changeRollerSpeed(0),
        floor.changeRollerSetpoint(0),
        turret.changeSetpoint(0),
        intake.changeExtSetpoint(0),
        intake.changeWheelSpeed(0),
        hood.changeSetpoint(0));
  }

  public Command intakeFuel() {
    return Commands.sequence(
        logMessage("Fuel Intake"),
        intake.changeExtSetpoint(6),
        Commands.waitSeconds(0.5),
        intake.changeWheelSpeed(0.7));
  }

  public Command turretRotation() {
    return Commands.sequence(logMessage("Turret Rotation"), turret.changeSetpoint(90));
  }

  public Command tunnelTravel() {
    return Commands.sequence(logMessage("Tunnel Travel"), tunnel.changeRollerSpeed(90));
  }

  public Command floorMove() {
    return Commands.sequence(floor.changeRollerSetpoint(10));
  }

  public Command hoodMove() {
    return Commands.sequence(hood.changeSetpoint(1));
  }
}
