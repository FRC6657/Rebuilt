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

  private TalonFX motor = new TalonFX(GlobalConstants.CAN.Floor_One.id);
  private VoltageOut setpoint = new VoltageOut(0);

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              FloorConstants.MOTOR, 0.001, FloorConstants.GEAR_RATIO),
          FloorConstants.MOTOR);

  public FloorIO_Sim() {
    motor
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
    motor.setControl(setpoint);

    // Sim Stuff
    var motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorSim.getMotorVoltage());
    motorModel.update(1 / GlobalConstants.mainLoopFrequency);
    motorSim.setRawRotorPosition(motorModel.getAngularPosition().times(FloorConstants.GEAR_RATIO));
    motorSim.setRotorVelocity(motorModel.getAngularVelocity().times(FloorConstants.GEAR_RATIO));

    // Log Data
    inputs.temp = -1; // Sim has no temps
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint.Output = setpoint;
  }
}
