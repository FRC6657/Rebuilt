package frc.robot.subsystems.tunnel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class TunnelIO_Real implements TunnelIO {

  TalonFX tunnelMotor = new TalonFX(TunnelConstants.TUNNEL_MOTOR);

  private double tunnelSetpoint = TunnelConstants.INITIAL_SETPOINT;

  public TunnelIO_Real() {

    var motorConfigurator = tunnelMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();

    motorConfigs.CurrentLimits = TunnelConstants.CURRENT_CONFIGS;

    var motorPosition = tunnelMotor.getPosition();
    var motorVelocity = tunnelMotor.getVelocity();
    var motorAcceleration = tunnelMotor.getAcceleration();
    var motorTemp = tunnelMotor.getDeviceTemp();
    var motorVoltage = tunnelMotor.getMotorVoltage();
    var motorCurrent = tunnelMotor.getSupplyCurrent();

    motorTemp.setUpdateFrequency(TunnelConstants.UPDATE_FREQUENCY);
    motorPosition.setUpdateFrequency(TunnelConstants.UPDATE_FREQUENCY);
    motorVelocity.setUpdateFrequency(TunnelConstants.UPDATE_FREQUENCY);
    motorAcceleration.setUpdateFrequency(TunnelConstants.UPDATE_FREQUENCY);
    motorVoltage.setUpdateFrequency(TunnelConstants.UPDATE_FREQUENCY);
    motorCurrent.setUpdateFrequency(TunnelConstants.UPDATE_FREQUENCY);

    tunnelMotor.setPosition(0);

    setSpeed(TunnelIOInputs.setpoint);
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {
    TunnelIOInputs.setpoint = tunnelSetpoint;
    TunnelIOInputs.velocity = tunnelMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setSpeed(double speed) {
    tunnelSetpoint = speed;
  }
}
