package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class ClimberConstants {

  public static final DCMotor CLIMBER_MOTOR = DCMotor.getFalcon500(2);

  public static final double GEAR_RATIO = 20;
  public static final double SPROCKET_PD = 1.8037;

  public static final double CONVERSION_FACTOR = (Math.PI * SPROCKET_PD);

  // TODO Figure Out Units
  public static final double MIN_HEIGHT = 0.0;
  public static final double MAX_HEIGHT = 1;
  public static final double LOW_SETPOINT = MIN_HEIGHT;
  public static final double HOOK_SETPOINT = MAX_HEIGHT - 0.5;
  public static final double DRIVEIN_SETPOINT = MAX_HEIGHT;

  public static final double HEIGHT_TOLERANCE = 1.0;

  public static final TalonFXConfiguration MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          .withSlot0(new Slot0Configs().withKP(10))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true));

  public class Pedal {

    public static final DCMotor PEDAL_MOTOR = DCMotor.getFalcon500(2);
    public static final double PEDAL_GEAR_RATIO = 20; // Find later bois

    public static final double PEDAL_MIN_ANGLE = 0.0;
    public static final double PEDAL_MAX_ANGLE = 120.0;
    public static final double ANGLE_TOLERANCE = 2.0;

    public static final TalonFXConfiguration PEDAL_MOTOR_CONFIGURATION =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(PEDAL_GEAR_RATIO))
            .withSlot0(new Slot0Configs().withKP(10))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withStatorCurrentLimit(60)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true));

    public static enum PedalSetpoint {
      PEDAL_HOME(0.0),
      COUNTER_PHASE(90.0);

      public final double degrees;

      private PedalSetpoint(double degrees) {
        this.degrees = degrees;
      }
    }
  }
}
