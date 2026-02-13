// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.GlobalConstants;

public class ClimberIO_Real implements ClimberIO {
  /** Creates a new ClimberIO_Real. */
  private TalonFX motor;

  private TalonFX motorTwo;

  private MotionMagicVoltage setpoint = new MotionMagicVoltage(0);

  public ClimberIO_Real() {

    motor = new TalonFX(GlobalConstants.CAN.Climber.id);
    motor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIGURATION);

    var motorVoltageSignal = motor.getMotorVoltage();
    var motorCurrentSignal = motor.getSupplyCurrent();
    var motorPositionSignal = motor.getPosition();
    var motorVelocitySignal = motor.getVelocity();
    var motorAccelerationSignal = motor.getAcceleration();

    motorVoltageSignal.setUpdateFrequency(50);
    motorCurrentSignal.setUpdateFrequency(50);
    motorPositionSignal.setUpdateFrequency(50);
    motorVelocitySignal.setUpdateFrequency(50);
    motorAccelerationSignal.setUpdateFrequency(50);

    motor.optimizeBusUtilization();

    // Pedal
    motorTwo = new TalonFX(GlobalConstants.CAN.Pedal.id);
    motorTwo.getConfigurator().apply(ClimberConstants.Pedal.PEDAL_MOTOR_CONFIGURATION);

    var motorTwoVoltageSignal = motorTwo.getMotorVoltage();
    var motorTwoCurrentSignal = motorTwo.getSupplyCurrent();
    var motorTwoPositionSignal = motorTwo.getPosition();
    var motorTwoVelocitySignal = motorTwo.getVelocity();
    var motorTwoAccelerationSignal = motorTwo.getAcceleration();

    motorTwoVoltageSignal.setUpdateFrequency(50);
    motorTwoCurrentSignal.setUpdateFrequency(50);
    motorTwoPositionSignal.setUpdateFrequency(50);
    motorTwoVelocitySignal.setUpdateFrequency(50);
    motorTwoAccelerationSignal.setUpdateFrequency(50);

    motorTwo.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // This method will be called once per scheduler run
    motor.setControl(setpoint);

    inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition =
        motor.getPosition().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.motorVelocity =
        motor.getVelocity().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.positionSetpoint = setpoint.Position * ClimberConstants.CONVERSION_FACTOR;

    // Pedal
    motorTwo.setControl(setpoint);

    inputs.motorTwoVoltage = motorTwo.getMotorVoltage().getValueAsDouble();
    inputs.motorTwoCurrent = motorTwo.getSupplyCurrent().getValueAsDouble();
    inputs.motorTwoPosition = motorTwo.getPosition().getValueAsDouble() * 360;
    inputs.motorTwoVelocity = motorTwo.getVelocity().getValueAsDouble() * 360;
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
    double degrees =
        MathUtil.clamp(
            newPedalSetpoint,
            ClimberConstants.Pedal.PEDAL_MIN_ANGLE,
            ClimberConstants.Pedal.PEDAL_MAX_ANGLE);
    setpoint.Position = degrees / 360.0;
  }
}
