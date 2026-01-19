package frc.robot.subsystems.floor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FloorConstants {

  public static final TalonFXConfiguration motorConfigs =
    new TalonFXConfiguration()
      .withMotorOutput(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.CounterClockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast))
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(20)
          .withStatorCurrentLimit(40)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimitEnable(true));

  public static enum RollerSetpoint {
    Off(0.0),
    In(6),
    Out(-12);

    public final double voltage;

    private RollerSetpoint(double voltage) {
      this.voltage = voltage;
    }
  }
}
