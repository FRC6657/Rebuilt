// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.GlobalConstants;

public class ClimberIO_Real implements ClimberIO {
  /** Creates a new ClimberIO_Real. */
  private TalonFX climberMotor;

  private TalonFX pedalMotor;

  // private MotionMagicVoltage setpoint = new MotionMagicVoltage(0);

  private VoltageOut climbVoltage = new VoltageOut(0);
  private VoltageOut pedalVoltage = new VoltageOut(0);

  public ClimberIO_Real() {

    climberMotor = new TalonFX(GlobalConstants.CAN.Climber.id);
    climberMotor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIGURATION);

    var climberMotorVoltageSignal = climberMotor.getMotorVoltage();
    var climberMotorCurrentSignal = climberMotor.getSupplyCurrent();
    var climberMotorPositionSignal = climberMotor.getPosition();
    var climberMotorVelocitySignal = climberMotor.getVelocity();
    var climberMotorAccelerationSignal = climberMotor.getAcceleration();

    climberMotorVoltageSignal.setUpdateFrequency(50);
    climberMotorCurrentSignal.setUpdateFrequency(50);
    climberMotorPositionSignal.setUpdateFrequency(50);
    climberMotorVelocitySignal.setUpdateFrequency(50);
    climberMotorAccelerationSignal.setUpdateFrequency(50);

    climberMotor.optimizeBusUtilization();

    // Pedal
    pedalMotor = new TalonFX(GlobalConstants.CAN.Pedal.id);
    pedalMotor.getConfigurator().apply(ClimberConstants.Pedal.PEDAL_MOTOR_CONFIGURATION);

    var pedalMotorVoltageSignal = pedalMotor.getMotorVoltage();
    var pedalMotorCurrentSignal = pedalMotor.getSupplyCurrent();
    var pedalMotorPositionSignal = pedalMotor.getPosition();
    var pedalMotorVelocitySignal = pedalMotor.getVelocity();
    var pedalMotorAccelerationSignal = pedalMotor.getAcceleration();

    pedalMotorVoltageSignal.setUpdateFrequency(50);
    pedalMotorCurrentSignal.setUpdateFrequency(50);
    pedalMotorPositionSignal.setUpdateFrequency(50);
    pedalMotorVelocitySignal.setUpdateFrequency(50);
    pedalMotorAccelerationSignal.setUpdateFrequency(50);

    pedalMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // This method will be called once per scheduler run

    climberMotor.setControl(climbVoltage);

    inputs.climberMotorVoltage = climberMotor.getMotorVoltage().getValueAsDouble();
    inputs.climberMotorCurrent = climberMotor.getSupplyCurrent().getValueAsDouble();
    inputs.climberMotorPosition =
        climberMotor.getPosition().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.climberMotorVelocity =
        climberMotor.getVelocity().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    // inputs.climberSetpoint = setpoint.Position * ClimberConstants.CONVERSION_FACTOR;

    // Pedal
    pedalMotor.setControl(pedalVoltage);

    inputs.pedalMotorVoltage = pedalMotor.getMotorVoltage().getValueAsDouble();
    inputs.pedalMotorCurrent = pedalMotor.getSupplyCurrent().getValueAsDouble();
    inputs.pedalMotorPosition = pedalMotor.getPosition().getValueAsDouble() * 360;
    inputs.pedalMotorVelocity = pedalMotor.getVelocity().getValueAsDouble() * 360;
    inputs.pedalMotorAcceleration = pedalMotor.getAcceleration().getValueAsDouble() * 360;
    // inputs.pedalSetpoint = setpoint.Position * 360;
  }

  // @Override
  // public void changeClimberSetpoint(double newClimberSetpoint) {
  //   var inches =
  //       MathUtil.clamp(
  //           newClimberSetpoint, ClimberConstants.MIN_HEIGHT, ClimberConstants.MAX_HEIGHT);
  //   setpoint.Position = inches / ClimberConstants.CONVERSION_FACTOR;
  // }

  // @Override
  // public void changePedalSetpoint(double newPedalSetpoint) {
  //   double degrees =
  //       MathUtil.clamp(
  //           newPedalSetpoint,
  //           ClimberConstants.Pedal.PEDAL_MIN_ANGLE,
  //           ClimberConstants.Pedal.PEDAL_MAX_ANGLE);
  //   setpoint.Position = degrees / 360.0;
  // }

  @Override
  public void changeClimberSetpoint(double newClimberSetpoint) {
    this.climbVoltage = new VoltageOut(newClimberSetpoint);
  }

  @Override
  public void changePedalSetpoint(double newPedalSetpoint) {
    pedalVoltage.Output = newPedalSetpoint;
  }
}
