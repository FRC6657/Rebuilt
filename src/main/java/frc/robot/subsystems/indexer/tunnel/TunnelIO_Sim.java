// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer.tunnel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;

/** Simulated tunnel indexer implementation using a DCMotorSim physics model. */
public class TunnelIO_Sim implements TunnelIO {

  private TalonFX tunnelMotor = new TalonFX(GlobalConstants.CAN.Tunnel.id);
  private VoltageOut setpoint = new VoltageOut(0);

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              TunnelConstants.MOTOR, 0.001, TunnelConstants.GEAR_RATIO),
          TunnelConstants.MOTOR);

  public TunnelIO_Sim() {
    tunnelMotor
        .getConfigurator()
        .apply(
            TunnelConstants.CONFIG.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(false)));
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {

    // Control the motor
    tunnelMotor.setControl(setpoint);

    // Sim Stuff
    var motorSim = tunnelMotor.getSimState();
    motorSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorSim.getMotorVoltage());
    motorModel.update(1 / GlobalConstants.mainLoopFrequency);
    motorSim.setRawRotorPosition(motorModel.getAngularPosition().times(TunnelConstants.GEAR_RATIO));
    motorSim.setRotorVelocity(motorModel.getAngularVelocity().times(TunnelConstants.GEAR_RATIO));

    // Log Data
    inputs.temp = -1; // Sim has no temps
    inputs.voltage = tunnelMotor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = tunnelMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint.Output = setpoint;
  }
}
