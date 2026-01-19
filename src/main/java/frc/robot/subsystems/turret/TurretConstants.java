package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
  public static final double INITIAL_SETPOINT = 0.0;
  // update gear ration
  public static final double GEAR_RATIO = 1 / 1d;
  public static final double supplyLimit = 30;
  public static final double statorLimit = 60;
  public static final CurrentLimitsConfigs currentConfigs =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(statorLimit)
          .withSupplyCurrentLimit(supplyLimit)
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLowerLimit(supplyLimit)
          .withSupplyCurrentLowerTime(0);
  public static Slot0Configs motorSlot0 =
      new Slot0Configs()
          .withKS(0) // Volts
          .withKG(0.0) // Volts
          .withGravityType(GravityTypeValue.Arm_Cosine)
          .withKP(100.0)
          .withKI(0)
          .withKD(0);
  public static double MaxVelocity = 390d / 360;
  public static double MaxAcceleration = 800d / 360;
  public static MotionMagicConfigs MotionMagicConfig =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(MaxVelocity)
          .withMotionMagicAcceleration(MaxAcceleration);

  // Visualization Stuff
  public static Translation3d TURRET_CENTER = new Translation3d(0.118317, -0.118317, 0.511175);
  public static Translation3d HOOD_OFFSET =
      new Translation3d(0, Units.inchesToMeters(-4.156585), Units.inchesToMeters(2.75));
}
