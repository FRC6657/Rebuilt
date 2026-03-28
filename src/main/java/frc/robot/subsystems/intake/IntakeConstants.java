package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Constants for the intake subsystem, split into Extension (linear actuator) and Roller (spinning
 * wheel) configurations.
 */
public class IntakeConstants {

  /** Constants for the intake extension mechanism (linear motion via Falcon 500). */
  public class Extension {

    public static final DCMotor MOTOR = DCMotor.getFalcon500(1);

    public static final double INITIAL_SETPOINT = 0; // in

    public static final double MIN_SETPOINT = 0; // in (fully retracted)
    public static final double MAX_SETPOINT = 12.375; // in (fully extended)
    public static final double SHUFFLE_SETPOINT = 5.0; // in (partially extended)
    public static final double POSITION_TOLERANCE = 0.5; // in

    public static final double SHUFFLE_PERIOD = 1.5;

    /** Overall gear reduction from motor to output. */
    public static final double GEAR_RATIO = (5d / 1d) * (36d / 28d) * (28d / 15d);

    /** Linear inches of travel per output shaft rotation (circumference of drive pulley). */
    public static final double CONVERSION_FACTOR = Math.PI; // Linear Inches Per Rotation

    public static final double SUPPLY_LIMIT = 30; // Amps
    public static final double STATOR_LIMIT = 60; // Amps

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
                    .withSupplyCurrentLimitEnable(true));

    /** Predefined extension setpoints with position, velocity, and acceleration profiles. */
    public static enum ExtensionSetpoint {
      RETRACTED_SLOW(MIN_SETPOINT + 1, 5, 40),
      RETRACTED_FAST(MIN_SETPOINT + 1, 80, 160),
      EXTENDED_SLOW(MAX_SETPOINT, 5, 40),
      EXTENDED_FAST(MAX_SETPOINT, 160, 320),
      SHUFFLE_IN(MIN_SETPOINT + 1, 80, 160),
      SHUFFLE_OUT(SHUFFLE_SETPOINT, 80, 160),
      Off(MIN_SETPOINT + 1, 0, 0);

      public final double position;
      public final double velocity;
      public final double acceleration;

      private ExtensionSetpoint(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
      }
    }
  }

  /** Constants for the intake roller (voltage-controlled spinning wheels). */
  public class Roller {

    public static final DCMotor MOTOR = DCMotor.getFalcon500(1);
    public static final double GEAR_RATIO = 24d / 11d;

    public static final TalonFXConfiguration CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withStatorCurrentLimit(80)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true));

    /** Predefined roller voltage setpoints. */
    public static final double Off = 0.0; // No power

    public static final double FORWARD = 12;
    public static final double REVERSE = -12; // 6V reverse (eject)
    public static final double SHUFFLE = 4;
  }
}
