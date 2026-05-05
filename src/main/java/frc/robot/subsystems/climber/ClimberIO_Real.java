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

  private VoltageOut hookVoltage = new VoltageOut(0);

  private PositionVoltage hookPositionVoltage = new PositionVoltage(0);

  private boolean hookUseRawVoltage = true;

  public ClimberIO_Real() {

    hookMotor = new TalonFX(GlobalConstants.CAN.Climber.id);
    hookMotor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIGURATION);

    var climberMotorVoltageSignal = hookMotor.getMotorVoltage();
    var climberMotorCurrentSignal = hookMotor.getSupplyCurrent();
    var climberMotorPositionSignal = hookMotor.getPosition();
    var climberMotorVelocitySignal = hookMotor.getVelocity();
    var climberMotorAccelerationSignal = hookMotor.getAcceleration();

    climberMotorVoltageSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    climberMotorCurrentSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    climberMotorPositionSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    climberMotorVelocitySignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    climberMotorAccelerationSignal.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    hookMotor.optimizeBusUtilization();

    hookMotor.setPosition(ClimberConstants.MAX_HEIGHT / ClimberConstants.CONVERSION_FACTOR);
    // hookMotor.setPosition(0);
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
  }

  @Override
  public void changeHookSetpoint(double setpoint, boolean rawVoltage) {
    this.hookUseRawVoltage = rawVoltage;
    this.hookVoltage.Output = setpoint;
    this.hookPositionVoltage.Position = setpoint / ClimberConstants.CONVERSION_FACTOR;
  }
}
