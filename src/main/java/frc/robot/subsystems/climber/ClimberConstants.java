package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
  public static final double MAX_HEIGHT = 17; // inches
  public static final double LOW_SETPOINT = MIN_HEIGHT;
  public static final double HOOK_SETPOINT = MAX_HEIGHT - 12.0;
  public static final double DRIVEIN_SETPOINT = MAX_HEIGHT;

  public static final double HEIGHT_TOLERANCE = 1.0;

  public static final TalonFXConfiguration MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          .withSlot0(new Slot0Configs().withKP(20))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true));
}
