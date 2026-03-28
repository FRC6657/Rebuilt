// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;

public class ClimberIO_Sim implements ClimberIO {

  private TalonFX motor = new TalonFX(GlobalConstants.CAN.Climber.id);

  private MotionMagicVoltage setpoint = new MotionMagicVoltage(0);

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ClimberConstants.CLIMBER_MOTOR, 0.001, ClimberConstants.GEAR_RATIO),
          ClimberConstants.CLIMBER_MOTOR);

  /** Creates a new ClimberIO_Sim. */
  public ClimberIO_Sim() {
    motor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIGURATION);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // This method will be called once per scheduler run

    // Apply setpoints
    motor.setControl(setpoint);

    // Simulate motor
    var motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorSim.getMotorVoltage());
    motorModel.update(0.02);
    motorSim.setRawRotorPosition(
        motorModel.getAngularPosition().times(ClimberConstants.GEAR_RATIO));
    motorSim.setRotorVelocity(motorModel.getAngularVelocity().times(ClimberConstants.GEAR_RATIO));

    // Update inputs
    inputs.climberMotorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.climberMotorCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.climberMotorPosition =
        motor.getPosition().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.climberMotorVelocity =
        motor.getVelocity().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.climberMotorAcceleration =
        motor.getAcceleration().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.climberSetpoint = setpoint.Position * ClimberConstants.CONVERSION_FACTOR;
  }

  @Override
  public void changeHookSetpoint(double newClimberSetpoint, boolean useRawVoltage) {
    var inches =
        MathUtil.clamp(
            newClimberSetpoint, ClimberConstants.MIN_HEIGHT, ClimberConstants.MAX_HEIGHT);
    setpoint.Position = inches / ClimberConstants.CONVERSION_FACTOR;
  }
}
