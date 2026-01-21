package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.GlobalConstants;

public class IntakeIO_Real implements IntakeIO {
    private TalonFX extMotor;
    private TalonFX wheelMotor;
    private Canandmag encoder;

    private double angleSetpoint = IntakeConstants.ExtensionMotor.maxLength;
    private double speedSetpoint = 0;

    private ProfiledPIDController extPID =
        new ProfiledPIDController(13.5, 0, 0, new Constraints(Units.inchesToMeters(12), Units.inchesToMeters(10))
    );

    public IntakeIO_Real() {
        extMotor = new TalonFX(GlobalConstants.CAN.Intake_Extension.id);
        wheelMotor = new TalonFX(GlobalConstants.CAN.Intake_Wheels.id);
        encoder = new Canandmag(GlobalConstants.CAN.Intake_Encoder.id);
        
        var extConfigurator = extMotor.getConfigurator();
        var extConfigs = new TalonFXConfiguration();
        extConfigs.CurrentLimits = IntakeConstants.ExtensionMotor.extCurrentConfigs;
        extConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extConfigurator.apply(extConfigs);
        extMotor.setNeutralMode(NeutralModeValue.Brake);

        var wheelConfigurator = wheelMotor.getConfigurator();
        var wheelConfigs = new TalonFXConfiguration();
        wheelConfigs.CurrentLimits = IntakeConstants.WheelMotor.wheelCurrentConfigs;
        wheelConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wheelConfigurator.apply(wheelConfigs);
        wheelMotor.setNeutralMode(NeutralModeValue.Brake);
        
        var extVelocitySignal = extMotor.getVelocity();
        var extTempSignal = extMotor.getDeviceTemp();
        var extVoltageSignal = extMotor.getMotorVoltage();
        var extCurrentSignal = extMotor.getSupplyCurrent();
        extVelocitySignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        extTempSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        extVoltageSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        extCurrentSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

        var wheelVelocitySignal = wheelMotor.getVelocity();
        var wheelTempSignal = wheelMotor.getDeviceTemp();
        var wheelVoltageSignal = wheelMotor.getMotorVoltage();
        var wheelCurrentSignal = wheelMotor.getSupplyCurrent();
        wheelVelocitySignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        wheelTempSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        wheelVoltageSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        wheelCurrentSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

        extMotor.optimizeBusUtilization();
        extPID.enableContinuousInput(0, 2);
        changeExtSetpoint(IntakeConstants.ExtensionMotor.minLength);
        extPID.reset(encoder.getAbsPosition());
        wheelMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.encoderAbsPosition = Units.metersToInches(encoder.getAbsPosition());
        inputs.encoderRelPosition = Units.metersToInches(encoder.getPosition());
        inputs.encoderVelocity = Units.metersToInches(encoder.getVelocity());

        inputs.extMotorVelocity = extMotor.getVelocity().getValueAsDouble();
        inputs.extMotorTemp = extMotor.getDeviceTemp().getValueAsDouble();
        inputs.extMotorVoltage = extMotor.get() * RobotController.getBatteryVoltage();
        inputs.extMotorCurrent = extMotor.getSupplyCurrent().getValueAsDouble();
        inputs.extMotorSetpoint = speedSetpoint;

        inputs.wheelMotorVelocity = wheelMotor.getVelocity().getValueAsDouble();
        inputs.wheelMotorTemp = wheelMotor.getDeviceTemp().getValueAsDouble();
        inputs.wheelMotorVoltage = wheelMotor.get() * RobotController.getBatteryVoltage();
        inputs.wheelMotorCurrent = wheelMotor.getSupplyCurrent().getValueAsDouble();

        double pidOutput = extPID.calculate(inputs.encoderAbsPosition, angleSetpoint);
        extMotor.set(pidOutput);

        Logger.recordOutput("Intake/ExtensionPIDOutput", pidOutput);
        Logger.recordOutput("Intake/ExtensionPIDProfileSetpoint", extPID.getSetpoint().position);
    }

    @Override
    public void changeExtSetpoint(double setpoint){
        speedSetpoint = setpoint;
    }
}
