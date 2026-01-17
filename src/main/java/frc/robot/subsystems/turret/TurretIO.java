// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  
  @AutoLog
  public static class TurretIOInputs {
    public double kSetpoint = 0.0;
    public double kVelocity = 0.0;
    public double kAccerleration = 0.0;
    public double kTemp = 0.0;
    public double kVoltage = 0.0;
    public double kCurrent = 0.0;
    public double kPosition = TurretConstants.INITIAL_SETPOINT;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void changeSetpoint(double setpoint) {}
}
