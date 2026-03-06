package frc.robot.subsystems.indexer.tunnel;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.GlobalConstants;

/** Real hardware implementation of the tunnel indexer using a TalonFX with voltage control. */
public class TunnelIO_Real implements TunnelIO {

  TalonFX motor = new TalonFX(GlobalConstants.CAN.Tunnel.id);
  VoltageOut setpoint = new VoltageOut(0);

  public TunnelIO_Real() {

    motor.getConfigurator().apply(TunnelConstants.CONFIG);

    var temp = motor.getDeviceTemp();
    var voltage = motor.getMotorVoltage();
    var statorCurrent = motor.getSupplyCurrent();

    temp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4d);
    voltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    statorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {

    // Control Motor
    motor.setControl(setpoint);

    // Log Data
    inputs.temp = motor.getDeviceTemp().getValueAsDouble();
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint.Output = setpoint;
  }
}
