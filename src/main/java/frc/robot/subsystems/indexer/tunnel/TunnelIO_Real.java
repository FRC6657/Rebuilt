package frc.robot.subsystems.indexer.tunnel;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.GlobalConstants;

/** Real hardware implementation of the tunnel indexer using a TalonFX with voltage control. */
public class TunnelIO_Real implements TunnelIO {

  TalonFX tunnelMotor = new TalonFX(GlobalConstants.CAN.Tunnel.id);
  VoltageOut setpoint = new VoltageOut(0);

  public TunnelIO_Real() {

    tunnelMotor.getConfigurator().apply(TunnelConstants.CONFIG);

    var temp = tunnelMotor.getDeviceTemp();
    var voltage = tunnelMotor.getMotorVoltage();
    var statorCurrent = tunnelMotor.getSupplyCurrent();

    temp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4d);
    voltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    statorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    tunnelMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {

    // Control Motor
    tunnelMotor.setControl(setpoint);

    // Log Data
    inputs.temp = tunnelMotor.getDeviceTemp().getValueAsDouble();
    inputs.statorCurrent = tunnelMotor.getStatorCurrent().getValueAsDouble();
    inputs.voltage = tunnelMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint.Output = setpoint;
  }
}
