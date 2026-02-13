// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIO_Sim implements ClimberIO {

  private TalonFX motor;
  private MotionMagicVoltage setpoint = new MotionMagicVoltage(0);

  private TalonFX motorTwo;

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ClimberConstants.CLIMBER_MOTOR, 0.001, ClimberConstants.GEAR_RATIO),
          ClimberConstants.CLIMBER_MOTOR);

  private DCMotorSim motorTwoModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ClimberConstants.Pedal.PEDAL_MOTOR, 0.001, ClimberConstants.Pedal.PEDAL_GEAR_RATIO),
          ClimberConstants.Pedal.PEDAL_MOTOR);

  /** Creates a new ClimberIO_Sim. */
  public ClimberIO_Sim() {

    motor = new TalonFX(ClimberConstants.MOTOR_CANID);
    motor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIGURATION);

    motorTwo = new TalonFX(ClimberConstants.Pedal.PEDAL_MOTOR_CANID);
    motorTwo.getConfigurator().apply(ClimberConstants.Pedal.PEDAL_MOTOR_CONFIGURATION);
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
    inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition =
        motor.getPosition().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.motorVelocity =
        motor.getVelocity().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.motorAcceleration =
        motor.getAcceleration().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.positionSetpoint = setpoint.Position * ClimberConstants.CONVERSION_FACTOR;

    // Pedal
    motorTwo.setControl(setpoint);

    var motorTwoSim = motorTwo.getSimState();
    motorTwoSim.setSupplyVoltage(12);
    motorTwoModel.setInputVoltage(motorTwoSim.getMotorVoltage());
    motorTwoModel.update(0.02);
    motorTwoSim.setRawRotorPosition(
        motorTwoModel.getAngularPosition().times(ClimberConstants.Pedal.PEDAL_GEAR_RATIO));
    motorTwoSim.setRotorVelocity(
        motorTwoModel.getAngularVelocity().times(ClimberConstants.Pedal.PEDAL_GEAR_RATIO));

    inputs.motorTwoVoltage = motorTwo.getMotorVoltage().getValueAsDouble();
    inputs.motorTwoCurrent = motorTwo.getSupplyCurrent().getValueAsDouble();
    inputs.motorTwoPosition = motorTwo.getPosition().getValueAsDouble() * 360;
    inputs.motorTwoVelocity = motorTwo.getVelocity().getValueAsDouble();
    inputs.motorTwoAcceleration = motorTwo.getAcceleration().getValueAsDouble() * 360;
    inputs.positionTwoSetpoint = setpoint.Position * 360;
  }

  @Override
  public void changeClimberSetpoint(double newClimberSetpoint) {
    var inches =
        MathUtil.clamp(
            newClimberSetpoint, ClimberConstants.MIN_HEIGHT, ClimberConstants.MAX_HEIGHT);
    setpoint.Position = inches / ClimberConstants.CONVERSION_FACTOR;
  }

  @Override
  public void changePedalSetpoint(double newPedalSetpoint) {
    var degrees =
        MathUtil.clamp(
            newPedalSetpoint,
            ClimberConstants.Pedal.PEDAL_MIN_ANGLE,
            ClimberConstants.Pedal.PEDAL_MAX_ANGLE);
    setpoint.Position = degrees / 360.0;
  }
}
