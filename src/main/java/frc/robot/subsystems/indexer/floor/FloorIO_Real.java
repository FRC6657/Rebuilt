package frc.robot.subsystems.indexer.floor;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.indexer.floor.FloorConstants.FloorSetpoint;

/** Real hardware implementation of the floor indexer using a TalonFX with voltage control. */
public class FloorIO_Real implements FloorIO {

  TalonFX motor = new TalonFX(GlobalConstants.CAN.Floor.id);
  VoltageOut setpoint = new VoltageOut(0);

  public FloorIO_Real() {

    motor.getConfigurator().apply(FloorConstants.CONFIG);

    // Temp status signals
    var temp = motor.getDeviceTemp();
    var voltage = motor.getMotorVoltage();
    var statorCurrent = motor.getStatorCurrent();

    // Set status frequencies
    temp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4);
    voltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    statorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    // Turn down not used status signals.
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FloorIOInputs inputs) {

    // Control Motor
    motor.setControl(setpoint);

    // Log Data
    inputs.temp = motor.getDeviceTemp().getValueAsDouble();
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(FloorSetpoint setpoint) {
    this.setpoint.Output = setpoint.voltage;
  }
}
