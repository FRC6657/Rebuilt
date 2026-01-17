package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static double updateFrequency = 50d;

    public static enum CAN {
        ExtensionMotor(21),
        Encoder(22);

        public int id;
        CAN(int id) {
            this.id = id;
        }
    }

    public static class ExtensionMotor {
        public static final double minLength = Units.inchesToMeters(0);
        public static final double maxLength = Units.inchesToMeters(6);

        public static final double extCurrentLimit = 30;

        public static final CurrentLimitsConfigs extCurrentConfigs =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(extCurrentLimit)
                .withSupplyCurrentLimit(extCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(30)
                .withSupplyCurrentLowerTime(0);
    }
}
