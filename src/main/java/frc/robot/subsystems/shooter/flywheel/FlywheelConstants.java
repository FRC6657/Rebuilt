package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the flywheel shooter, including motor config, limits, and PID gains. */
public class FlywheelConstants {

  public static final DCMotor MOTOR = DCMotor.getFalcon500(1);
  public static final double GEAR_RATIO = 1.0; // Direct drive (1:1)
  public static final double VELOCITY_TOLERANCE = 100; // RPM tolerance for "at setpoint" check

  public static final double SUPPLY_LIMIT = 60; // Amps
  public static final double STATOR_LIMIT = 80; // Amps

  public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(STATOR_LIMIT)
                  .withSupplyCurrentLimit(SUPPLY_LIMIT)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(SUPPLY_LIMIT)
                  .withSupplyCurrentLowerTime(0))
          .withSlot0(new Slot0Configs().withKV(12d / (6380d / 60d)).withKP(0).withKD(0));
}
