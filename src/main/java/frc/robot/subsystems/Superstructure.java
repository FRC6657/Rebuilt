package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.turret.TurretConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  // Subsystems
  Drivebase drivebase;

  // Fake test visualization angles (radians)
  private double turretAngle = 0.0;
  private double hoodAngle = 0.0;

  public Superstructure(Drivebase drivebase) {
    this.drivebase = drivebase;
  }

  @AutoLogOutput(key = "3DComponents")
  public Pose3d[] get3DComponents() {

    //Turret / Shooter
    Rotation3d turretRotation = new Rotation3d(0, 0, turretAngle);
    Translation3d rotatedHoodOffset = TurretConstants.HOOD_OFFSET.rotateBy(turretRotation);
    Translation3d hoodPosition = TurretConstants.TURRET_CENTER.plus(rotatedHoodOffset);
    Rotation3d hoodLocalPitch = new Rotation3d(hoodAngle, 0, 0);
    Rotation3d hoodRotation = hoodLocalPitch.rotateBy(turretRotation);

    return new Pose3d[] {
      new Pose3d(TurretConstants.TURRET_CENTER, turretRotation), new Pose3d(hoodPosition, hoodRotation)
    };
  }

  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }
}
