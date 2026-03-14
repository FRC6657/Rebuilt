// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer.floor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;

/** Simulated floor indexer implementation using a DCMotorSim physics model. */
public class FloorIO_Sim implements FloorIO {

  private TalonFX motorOne = new TalonFX(GlobalConstants.CAN.Floor_One.id);
  private TalonFX motorTwo = new TalonFX(GlobalConstants.CAN.Floor_Two.id);
  private VoltageOut setpoint = new VoltageOut(0);

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              FloorConstants.MOTOR, 0.001, FloorConstants.GEAR_RATIO),
          FloorConstants.MOTOR);

  public FloorIO_Sim() {
    motorOne
        .getConfigurator()
        .apply(
            FloorConstants.CONFIG.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(false)));
  }

  @Override
  public void updateInputs(FloorIOInputs inputs) {

    // Control the motor
    motorOne.setControl(setpoint);

    // Sim Stuff
    var motorOneSim = motorOne.getSimState();
    motorOneSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorOneSim.getMotorVoltage());
    motorModel.update(1 / GlobalConstants.mainLoopFrequency);
    motorOneSim.setRawRotorPosition(
        motorModel.getAngularPosition().times(FloorConstants.GEAR_RATIO));
    motorOneSim.setRotorVelocity(motorModel.getAngularVelocity().times(FloorConstants.GEAR_RATIO));

    var motorTwoSim = motorTwo.getSimState();
    motorTwoSim.setSupplyVoltage(12);
    motorTwoSim.setRawRotorPosition(
        motorModel.getAngularPosition().times(FloorConstants.GEAR_RATIO));
    motorTwoSim.setRotorVelocity(motorModel.getAngularVelocity().times(FloorConstants.GEAR_RATIO));

    // Log Data
    inputs.motorOneTemp = -1; // Sim has no temps
    inputs.motorOneVoltage = motorOne.getMotorVoltage().getValueAsDouble();
    inputs.motorOneStatorCurrent = motorOne.getStatorCurrent().getValueAsDouble();

    inputs.motorTwoTemp = -1; // Sim has no temps
    inputs.motorTwoVoltage = motorOne.getMotorVoltage().getValueAsDouble();
    inputs.motorTwoStatorCurrent = motorOne.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint.Output = setpoint;
  }
}
