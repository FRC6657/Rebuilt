// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
    public double motorPosition = 0.0;
    public double motorVelocity = 0.0;
    public double motorAcceleration = 0.0;
    public double positionSetpoint = 0.0;
    public double motorTwoVoltage = 0.0;
    public double motorTwoCurrent = 0.0;
    public double motorTwoPosition = 0.0;
    public double motorTwoVelocity = 0.0;
    public double motorTwoAcceleration = 0.0;
    public double positionTwoSetpoint = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void changeSetpoint(double newSetpoint) {}
}
