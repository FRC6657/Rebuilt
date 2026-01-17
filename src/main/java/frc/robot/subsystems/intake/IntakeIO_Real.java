package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class IntakeIO_Real {
    private TalonFX extMotor;

    private Canandmag encoder;

    private double angleSetpoint = IntakeConstants.ExtensionMotor.maxLength;
    private double speedSetpoint = 0;

    public IntakeIO_Real() {
        extMotor = new TalonFX(IntakeConstants.CAN.ExtensionMotor.id);
        encoder = new Canandmag(IntakeConstants.CAN.Encoder.id);
        
        var extConfigurator = extMotor.getConfigurator();
        var extConfigs = new TalonFXConfiguration();
        extConfigs.CurrentLimits = IntakeConstants.ExtensionMotor.extCurrentConfigs;
        extConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extConfigurator.apply(extConfigs);
        extMotor.setNeutralMode(NeutralModeValue.Brake);
        
        var extVelocitySignal = extMotor.getVelocity();
        var extTempSignal = extMotor.getDeviceTemp();
        var extVoltageSignal = extMotor.getMotorVoltage();
        var extCurrentSignal = extMotor.getSupplyCurrent();

        extMotor.optimizeBusUtilization();

        changeExtSeptoint(IntakeConstants.ExtensionMotor.minLength);
    }

}
