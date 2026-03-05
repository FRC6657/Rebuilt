// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double climberMotorVoltage = 0.0;
    public double climberMotorCurrent = 0.0;
    public double climberMotorPosition = 0.0;
    public double climberMotorVelocity = 0.0;
    public double climberMotorAcceleration = 0.0;
    public double climberSetpoint = 0.0;

    public double pedalMotorVoltage = 0.0;
    public double pedalMotorCurrent = 0.0;
    public double pedalMotorPosition = 0.0;
    public double pedalMotorVelocity = 0.0;
    public double pedalMotorAcceleration = 0.0;
    public double pedalSetpoint = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void changeHookSetpoint(double setpoint, boolean useRawVoltage) {}

  public default void changePedalSetpoint(double setpoint) {}
}
