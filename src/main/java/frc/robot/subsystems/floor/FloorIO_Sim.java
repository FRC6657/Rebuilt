// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.floor;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.floor.FloorConstants.RollerSetpoint;

public class FloorIO_Sim implements FloorIO {
  /** Creates a new FloorIO_Sim. */
  private TalonFX motor;
  private MotionMagicVoltage setpoint = new MotionMagicVoltage(0);

  private DCMotorSim motorModel =
    new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        FloorConstants.MOTOR, 0.001, FloorConstants.GEAR_RATIO), 
      FloorConstants.MOTOR);

  public FloorIO_Sim() {

    motor = new TalonFX(1); //Replace with CANID
    motor.getConfigurator().apply(FloorConstants.motorConfigs);
  }

  @Override
  public void updateInputs(FloorIOInputs inputs) {

    motor.setControl(setpoint);

    var motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorSim.getMotorVoltage());
    motorModel.update(0.02);
    motorSim.setRawRotorPosition(motorModel.getAngularPosition().times(FloorConstants.GEAR_RATIO));
    motorSim.setRotorVelocity(motorModel.getAngularVelocity().times(FloorConstants.GEAR_RATIO));

    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.current = motor.getSupplyCurrent().getValueAsDouble();
    inputs.position =
      motor.getPosition().getValueAsDouble() * 360;
    inputs.velocity = motor.getVelocity().getValueAsDouble() * 360;
    inputs.acceleration =
      motor.getAcceleration().getValueAsDouble() *360;
    inputs.setpoint = setpoint.Position * 360;
  }

  @Override
  public void changeSetpoint(RollerSetpoint newSetpoint) {
    var degrees =
      MathUtil.clamp(newSetpoint.voltage, FloorConstants.MIN_ANGLE, FloorConstants.MAX_ANGLE);
    setpoint.Position = degrees / 360.0;
  }
}
