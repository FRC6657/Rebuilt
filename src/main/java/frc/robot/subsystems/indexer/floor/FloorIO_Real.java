package frc.robot.subsystems.indexer.floor;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.GlobalConstants;

/** Real hardware implementation of the floor indexer using a TalonFX with voltage control. */
public class FloorIO_Real implements FloorIO {

  TalonFX motorOne = new TalonFX(GlobalConstants.CAN.Floor.id);
  TalonFX motorTwo = new TalonFX(GlobalConstants.CAN.Ceiling_Wall.id);
  VoltageOut setpoint = new VoltageOut(0);

  public FloorIO_Real() {

    motorOne.getConfigurator().apply(FloorConstants.CONFIG);
    motorTwo.getConfigurator().apply(FloorConstants.CONFIG);

    motorTwo.setControl(
        new Follower(GlobalConstants.CAN.Floor.id, MotorAlignmentValue.Opposed));

    // Temp status signals
    var motorOneTemp = motorOne.getDeviceTemp();
    var motorOneVoltage = motorOne.getMotorVoltage();
    var motorOneStatorCurrent = motorOne.getStatorCurrent();
    var motorTwoTemp = motorTwo.getDeviceTemp();
    var motorTwoVoltage = motorTwo.getMotorVoltage();
    var motorTwoStatorCurrent = motorTwo.getStatorCurrent();

    // Set status frequencies
    motorOneTemp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4);
    motorOneVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    motorOneStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    motorTwoTemp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4);
    motorTwoVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    motorTwoStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
  }

  @Override
  public void updateInputs(FloorIOInputs inputs) {

    // Control Motor
    motorOne.setControl(setpoint);

    // Log Data
    inputs.motorOneTemp = motorOne.getDeviceTemp().getValueAsDouble();
    inputs.motorOneStatorCurrent = motorOne.getStatorCurrent().getValueAsDouble();
    inputs.motorOneVoltage = motorOne.getMotorVoltage().getValueAsDouble();
    inputs.motorTwoTemp = motorTwo.getDeviceTemp().getValueAsDouble();
    inputs.motorTwoStatorCurrent = motorTwo.getStatorCurrent().getValueAsDouble();
    inputs.motorTwoVoltage = motorTwo.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint.Output = setpoint;
  }
}
