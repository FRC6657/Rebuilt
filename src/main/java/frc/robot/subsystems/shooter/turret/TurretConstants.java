package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the turret rotation mechanism, including gear ratio, limits, PID config, and 3D
 * visualization offsets.
 */
public class TurretConstants {

  public static final DCMotor MOTOR = DCMotor.getFalcon500(1);

  public static final double ORIGIN_POSITION = 0.0;
  public static double restingSetpoint =
      195.0; // degrees (where the turret will go when it is not tracking)

  public static final double POSITION_TOLERANCE = 2.0; // degrees

  public static final double FULL_ROTATION_RANGE = 420.0; // degrees
  public static final double ROTATION_RANGE = FULL_ROTATION_RANGE - 20.0; // degrees

  /** Total gear reduction from motor to turret output. */
  public static final double GEAR_RATIO = (140d / 24d) * (66d / 14d);

  /** Degrees of turret travel per output shaft rotation. */
  public static final double CONVERSION_FACTOR = 360.0; // Degrees Per Rotation

  public static final double SUPPLY_LIMIT = 30; // Amps
  public static final double STATOR_LIMIT = 60; // Amps

  public static final double KP = 25.0;
  public static final double KD = 0;

  public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          .withSlot0(new Slot0Configs().withKP(KP).withKD(KD))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(STATOR_LIMIT)
                  .withSupplyCurrentLimit(SUPPLY_LIMIT)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(SUPPLY_LIMIT)
                  .withSupplyCurrentLowerTime(0));

  private static final double MOTOR_KV_ROT =
      12.0 * GEAR_RATIO * 2.0 * Math.PI / MOTOR.freeSpeedRadPerSec;

  public static final double FF_VOLTS_PER_DEG_SEC = (MOTOR_KV_ROT + KD) / 360.0;

  // 3D visualization offsets (from CAD model, in meters)
  /** Position of the turret center relative to the robot origin. */
  public static final Translation3d TURRET_CENTER =
      new Translation3d(0.118317, -0.105617, 0.511175);

  public static final Transform2d robotToTurret =
      new Transform2d(TURRET_CENTER.toTranslation2d(), new Rotation2d());

  /** Offset from turret center to the hood pivot point (rotates with turret). */
  public static final Translation3d HOOD_OFFSET =
      new Translation3d(0, Units.inchesToMeters(-4.7), Units.inchesToMeters(0.95));
}
