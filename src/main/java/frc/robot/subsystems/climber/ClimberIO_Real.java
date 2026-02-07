// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberIO_Real implements ClimberIO {
  /** Creates a new ClimberIO_Real. */
  private TalonFX motor;
  private MotionMagicVoltage setpoint = new MotionMagicVoltage(0);

  public ClimberIO_Real() {
    
    motor = new TalonFX(ClimberConstants.MOTOR_CANID);
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
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // This method will be called once per scheduler run
    motor.setControl(setpoint);

    inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition =
      motor.getPosition().getValueAsDouble()
        * ClimberConstants.CONVERSION_FACTOR;
    inputs.motorVelocity =
      motor.getVelocity().getValueAsDouble()
        * ClimberConstants.CONVERSION_FACTOR;
    inputs.positionSetpoint =
      setpoint.Position * ClimberConstants.CONVERSION_FACTOR;
  }

  @Override
  public void changeSetpoint(double newSetpoint) {
    var inches =
      MathUtil.clamp(
        newSetpoint, ClimberConstants.MIN_HEIGHT, ClimberConstants.MAX_HEIGHT);
      setpoint.Position =
        inches / ClimberConstants.CONVERSION_FACTOR;
  }
}
