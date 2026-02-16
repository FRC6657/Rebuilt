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
  private TalonFX pedalMotor = new TalonFX(GlobalConstants.CAN.Pedal.id);

  private MotionMagicVoltage setpoint = new MotionMagicVoltage(0);

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ClimberConstants.CLIMBER_MOTOR, 0.001, ClimberConstants.GEAR_RATIO),
          ClimberConstants.CLIMBER_MOTOR);

  private DCMotorSim pedalMotorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ClimberConstants.Pedal.PEDAL_MOTOR, 0.001, ClimberConstants.Pedal.PEDAL_GEAR_RATIO),
          ClimberConstants.Pedal.PEDAL_MOTOR);

  /** Creates a new ClimberIO_Sim. */
  public ClimberIO_Sim() {
    motor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIGURATION);
    pedalMotor.getConfigurator().apply(ClimberConstants.Pedal.PEDAL_MOTOR_CONFIGURATION);
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

    // Pedal
    pedalMotor.setControl(setpoint);

    var pedalMotorSim = pedalMotor.getSimState();
    pedalMotorSim.setSupplyVoltage(12);
    pedalMotorModel.setInputVoltage(pedalMotorSim.getMotorVoltage());
    pedalMotorModel.update(0.02);
    pedalMotorSim.setRawRotorPosition(
        pedalMotorModel.getAngularPosition().times(ClimberConstants.Pedal.PEDAL_GEAR_RATIO));
    pedalMotorSim.setRotorVelocity(
        pedalMotorModel.getAngularVelocity().times(ClimberConstants.Pedal.PEDAL_GEAR_RATIO));

    inputs.pedalMotorVoltage = pedalMotor.getMotorVoltage().getValueAsDouble();
    inputs.pedalMotorCurrent = pedalMotor.getSupplyCurrent().getValueAsDouble();
    inputs.pedalMotorPosition = pedalMotor.getPosition().getValueAsDouble() * 360;
    inputs.pedalMotorVelocity = pedalMotor.getVelocity().getValueAsDouble();
    inputs.pedalMotorAcceleration = pedalMotor.getAcceleration().getValueAsDouble() * 360;
    inputs.pedalSetpoint = setpoint.Position * 360;
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
