package frc.robot.subsystems.indexer.floor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the floor indexer, including motor config and voltage setpoints. */
public class FloorConstants {

  public static final double Off = 0.0; // No power
  public static final double FORWARD = 6; // 6V forward (move game pieces toward tunnel)
  public static final double REVERSE = -6; // 6V reverse (eject game pieces)

  public final double voltage = 0.0;

  public static final DCMotor MOTOR = DCMotor.getFalcon500(1);
  public static final double GEAR_RATIO = 32d / 11d; // Motor rotations per output rotation

  public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(20)
                  .withStatorCurrentLimit(40)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true));
  }
