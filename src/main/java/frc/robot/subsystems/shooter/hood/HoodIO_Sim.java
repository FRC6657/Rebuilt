// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;

/** Simulated hood implementation using a DCMotorSim physics model with TalonFX position PID. */
public class HoodIO_Sim implements HoodIO {

  private TalonFX motor = new TalonFX(GlobalConstants.CAN.Hood.id);
  private PositionVoltage positionVoltage =
      new PositionVoltage(HoodConstants.INITIAL_SETPOINT / HoodConstants.CONVERSION_FACTOR);

  private DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(HoodConstants.MOTOR, 0.0001, HoodConstants.GEAR_RATIO),
          HoodConstants.MOTOR);

  public HoodIO_Sim() {
    motor.getConfigurator().apply(HoodConstants.CONFIG);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    motor.setControl(positionVoltage);

    var simState = motor.getSimState();
    simState.setSupplyVoltage(12);
    motorSim.setInputVoltage(simState.getMotorVoltage());
    motorSim.update(1 / GlobalConstants.mainLoopFrequency);
    simState.setRawRotorPosition(motorSim.getAngularPosition().times(HoodConstants.GEAR_RATIO));
    simState.setRotorVelocity(motorSim.getAngularVelocity().times(HoodConstants.GEAR_RATIO));

    inputs.position = motor.getPosition().getValueAsDouble() * HoodConstants.CONVERSION_FACTOR;
    inputs.temp = 0;
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    positionVoltage.Position =
        MathUtil.clamp(setpoint, HoodConstants.MIN_SETPOINT, HoodConstants.MAX_SETPOINT)
            / HoodConstants.CONVERSION_FACTOR;
  }

  @Override
  public boolean atSetpoint() {
    return MathUtil.isNear(
        positionVoltage.Position * HoodConstants.CONVERSION_FACTOR,
        motor.getPosition().getValueAsDouble() * HoodConstants.CONVERSION_FACTOR,
        HoodConstants.POSITION_TOLERANCE);
  }
}
