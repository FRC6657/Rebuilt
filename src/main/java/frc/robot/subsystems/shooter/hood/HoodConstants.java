package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the hood angle mechanism, including limits, gear ratio, and PID config. */
public class HoodConstants {

  public static final DCMotor MOTOR = DCMotor.getFalcon500(1);

  public static final double INITIAL_SETPOINT = 10.0; // degrees (startup/home position)

  public static final double MIN_SETPOINT = 10.0; // degrees (lowest angle)
  public static final double MAX_SETPOINT = 40.0; // degrees (highest angle)
  public static final double POSITION_TOLERANCE = 0.25; // degrees

  /** Total gear reduction from motor to hood output. */
  public static final double GEAR_RATIO = (70d / 14d) * (180d / 10d);

  /** Degrees of hood travel per output shaft rotation. */
  public static final double CONVERSION_FACTOR = 360.0; // Degrees Per Rotation

  public static final double SUPPLY_LIMIT = 30; // Amps
  public static final double STATOR_LIMIT = 60; // Amps

  public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          .withSlot0(new Slot0Configs().withKS(0).withKP(250.0).withKD(0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(STATOR_LIMIT)
                  .withSupplyCurrentLimit(SUPPLY_LIMIT)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(SUPPLY_LIMIT)
                  .withSupplyCurrentLowerTime(0));
}
