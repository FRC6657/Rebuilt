package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.GlobalConstants;

public class HoodIO_Real implements HoodIO {

  // update placeholder value here
  TalonFX hoodMotor = new TalonFX(GlobalConstants.CAN.Hood.id);
  private MotionMagicVoltage motionMagicVoltage =
      new MotionMagicVoltage(HoodConstants.INITIAL_SETPOINT / 360);

  public HoodIO_Real() {

    var motorConfigurator = hoodMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.Slot0 = HoodConstants.motorSlot0;
    motorConfigs.CurrentLimits = HoodConstants.currentConfigs;
    motorConfigs.MotionMagic = HoodConstants.MotionMagicConfig;
    motorConfigurator.apply(motorConfigs);

    var Position = hoodMotor.getPosition();
    var Temp = hoodMotor.getDeviceTemp();
    var Voltage = hoodMotor.getMotorVoltage();
    var Current = hoodMotor.getSupplyCurrent();

    // update placeholder values
    Temp.setUpdateFrequency(50);
    Voltage.setUpdateFrequency(50);
    Current.setUpdateFrequency(50);
    Position.setUpdateFrequency(50);

    hoodMotor.optimizeBusUtilization();

    hoodMotor.setPosition(HoodConstants.INITIAL_SETPOINT / 360);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    inputs.Temp = hoodMotor.getDeviceTemp().getValueAsDouble();
    inputs.Current = hoodMotor.getSupplyCurrent().getValueAsDouble();
    inputs.Voltage = hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.Position = hoodMotor.getPosition().getValueAsDouble() * 360;
    inputs.Velocity = hoodMotor.getVelocity().getValueAsDouble() * 360;
    inputs.Accerleration = hoodMotor.getAcceleration().getValueAsDouble() * 360;
    inputs.Setpoint = motionMagicVoltage.Position * 360;

    hoodMotor.setControl(motionMagicVoltage);
  }

  @Override
  public void changeSetpoint(double setpoint) {
    motionMagicVoltage.Position =
        MathUtil.clamp(setpoint, HoodConstants.INITIAL_SETPOINT, HoodConstants.MAX_SETPOINT) / 360;
  }
}
