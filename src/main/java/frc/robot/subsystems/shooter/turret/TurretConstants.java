package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the turret rotation mechanism, including gear ratio, limits, PID config, and 3D
 * visualization offsets.
 */
public class TurretConstants {

  public static final DCMotor MOTOR = DCMotor.getFalcon500(1);

  public static final double INITIAL_SETPOINT = 0.0; // degrees (home/startup position)

  public static final double POSITION_TOLERANCE = 2.0; // degrees

  public static final double FULL_ROTATION_RANGE = 460.0; // degrees
  public static final double ROTATION_RANGE = FULL_ROTATION_RANGE - 50.0; // degrees

  /** Total gear reduction from motor to turret output. */
  public static final double GEAR_RATIO = (140d / 24d) * (66d / 14d);
  /** Degrees of turret travel per output shaft rotation. */
  public static final double CONVERSION_FACTOR = 360.0; // Degrees Per Rotation

  public static final double SUPPLY_LIMIT = 30; // Amps
  public static final double STATOR_LIMIT = 60; // Amps

  public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          .withSlot0(new Slot0Configs().withKP(25.0).withKD(2))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(STATOR_LIMIT)
                  .withSupplyCurrentLimit(SUPPLY_LIMIT)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(SUPPLY_LIMIT)
                  .withSupplyCurrentLowerTime(0));

  // 3D visualization offsets (from CAD model, in meters)
  /** Position of the turret center relative to the robot origin. */
  public static final Translation3d TURRET_CENTER =
      new Translation3d(0.118317, -0.105617, 0.511175);
  /** Offset from turret center to the hood pivot point (rotates with turret). */
  public static final Translation3d HOOD_OFFSET =
      new Translation3d(0, Units.inchesToMeters(-4.7), Units.inchesToMeters(0.95));
}
