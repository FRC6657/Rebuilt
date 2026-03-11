// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.GlobalConstants;

public class ClimberIO_Real implements ClimberIO {

  private TalonFX hookMotor;
  private TalonFX pedalMotor;

  private VoltageOut hookVoltage = new VoltageOut(0);

  private PositionVoltage hookPositionVoltage = new PositionVoltage(0);
  private PositionVoltage pedalPositionVoltage = new PositionVoltage(0);

  private boolean hookUseRawVoltage = true;

  public ClimberIO_Real() {

    hookMotor = new TalonFX(GlobalConstants.CAN.Climber.id);
    hookMotor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIGURATION);

    var climberMotorVoltageSignal = hookMotor.getMotorVoltage();
    var climberMotorCurrentSignal = hookMotor.getSupplyCurrent();
    var climberMotorPositionSignal = hookMotor.getPosition();
    var climberMotorVelocitySignal = hookMotor.getVelocity();
    var climberMotorAccelerationSignal = hookMotor.getAcceleration();

    climberMotorVoltageSignal.setUpdateFrequency(50);
    climberMotorCurrentSignal.setUpdateFrequency(50);
    climberMotorPositionSignal.setUpdateFrequency(50);
    climberMotorVelocitySignal.setUpdateFrequency(50);
    climberMotorAccelerationSignal.setUpdateFrequency(50);

    hookMotor.optimizeBusUtilization();

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

    hookMotor.setPosition(ClimberConstants.MAX_HEIGHT / ClimberConstants.CONVERSION_FACTOR);
    // hookMotor.setPosition(0);
    pedalMotor.setPosition(0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    hookMotor.setControl(hookUseRawVoltage ? hookVoltage : hookPositionVoltage);

    inputs.climberMotorVoltage = hookMotor.getMotorVoltage().getValueAsDouble();
    inputs.climberMotorCurrent = hookMotor.getSupplyCurrent().getValueAsDouble();
    inputs.climberMotorPosition =
        hookMotor.getPosition().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;
    inputs.climberMotorVelocity =
        hookMotor.getVelocity().getValueAsDouble() * ClimberConstants.CONVERSION_FACTOR;

    // Pedal
    pedalMotor.setControl(pedalPositionVoltage);

    inputs.pedalMotorVoltage = pedalMotor.getMotorVoltage().getValueAsDouble();
    inputs.pedalMotorCurrent = pedalMotor.getSupplyCurrent().getValueAsDouble();
    inputs.pedalMotorPosition = pedalMotor.getPosition().getValueAsDouble() * 360;
    inputs.pedalMotorVelocity = pedalMotor.getVelocity().getValueAsDouble() * 360;
    inputs.pedalMotorAcceleration = pedalMotor.getAcceleration().getValueAsDouble() * 360;
  }

  @Override
  public void changeHookSetpoint(double setpoint, boolean rawVoltage) {
    this.hookUseRawVoltage = rawVoltage;
    this.hookVoltage.Output = setpoint;
    this.hookPositionVoltage.Position = setpoint / ClimberConstants.CONVERSION_FACTOR;
  }

  @Override
  public void changePedalSetpoint(double setpoint) {
    pedalPositionVoltage.Position = setpoint / 360d;
  }
}
