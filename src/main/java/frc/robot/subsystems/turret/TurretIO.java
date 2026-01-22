// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double Setpoint = 0.0;
    public double Velocity = 0.0;
    public double Accerleration = 0.0;
    public double Temp = 0.0;
    public double Voltage = 0.0;
    public double Current = 0.0;
    public double Position = TurretConstants.INITIAL_SETPOINT;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void changeSetpoint(double setpoint) {}
}
