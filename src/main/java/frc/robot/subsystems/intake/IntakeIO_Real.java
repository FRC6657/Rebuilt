package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class IntakeIO_Real {
    private TalonFX extMotor;

    private double angleSetpoint = IntakeConstants.ExtensionMotor.maxLength;
    private double speedSetpoint = 0;

    private ProfiledPIDController extPID =
        new ProfiledPIDController(
            13.5, 0, 0, new Constraints(Units.inchesToMeters(12), Units.inchesToMeters(10))
    );

    public IntakeIO_Real() {
        var extConfigurator = extMotor.getConfigurator();
        var extConfigs = new TalonFXConfiguration();
        extConfigs.CurrentLimits = IntakeConstants.ExtensionMotor.extCurrentConfigs;
    }
}
