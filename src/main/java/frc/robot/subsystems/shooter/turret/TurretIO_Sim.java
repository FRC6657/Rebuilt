// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;

/** Simulated turret implementation using a DCMotorSim physics model with TalonFX position PID. */
public class TurretIO_Sim implements TurretIO {

  private double position = 0.0; // in degrees
  private TalonFX motor = new TalonFX(GlobalConstants.CAN.Turret.id);
  private PositionVoltage setpoint =
      new PositionVoltage(TurretConstants.INITIAL_SETPOINT / TurretConstants.CONVERSION_FACTOR);

  private DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              TurretConstants.MOTOR, 0.05, TurretConstants.GEAR_RATIO),
          TurretConstants.MOTOR);

  public TurretIO_Sim() {
    motor
        .getConfigurator()
        .apply(
            TurretConstants.CONFIG.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(false)));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    motor.setControl(setpoint);

    var simState = motor.getSimState();
    simState.setSupplyVoltage(12);
    motorSim.setInputVoltage(simState.getMotorVoltage());
    motorSim.update(1 / GlobalConstants.mainLoopFrequency);
    simState.setRawRotorPosition(motorSim.getAngularPosition().times(TurretConstants.GEAR_RATIO));
    simState.setRotorVelocity(motorSim.getAngularVelocity().times(TurretConstants.GEAR_RATIO));

    position = motor.getPosition().getValueAsDouble() * TurretConstants.CONVERSION_FACTOR;
    inputs.position = position;
    inputs.velocity = motor.getVelocity().getValueAsDouble() * TurretConstants.CONVERSION_FACTOR;
    inputs.acceleration =
        motor.getAcceleration().getValueAsDouble() * TurretConstants.CONVERSION_FACTOR;
    inputs.temp = 0;
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    // Wrap the input angle to [0, 360) degrees
    double clampedInput = setpoint % 360;
    if (clampedInput < 0) {
      clampedInput = clampedInput + 360;
    }

    if (Math.abs(clampedInput - position) > 180) {
      /*if the desired position is that far away from the current position, then we want to check if we can go the other way! */
      if (clampedInput + 360 < TurretConstants.ROTATION_RANGE) {
        clampedInput += 360;
      }
    }

    // Add initial offset and convert degrees to motor rotations
    this.setpoint.Position =
        (clampedInput + TurretConstants.INITIAL_SETPOINT) / TurretConstants.CONVERSION_FACTOR;
    this.setpoint.FeedForward = 0;
  }

  @Override
  public void changeSetpoint(double setpoint, double feedforwardDegPerSec) {
    changeSetpoint(setpoint);
    this.setpoint.FeedForward = feedforwardDegPerSec * TurretConstants.FF_VOLTS_PER_DEG_SEC;
  }

  @Override
  public boolean atSetpoint() {
    return MathUtil.isNear(
        setpoint.Position * TurretConstants.CONVERSION_FACTOR,
        motor.getPosition().getValueAsDouble() * TurretConstants.CONVERSION_FACTOR,
        TurretConstants.POSITION_TOLERANCE);
  }
}
