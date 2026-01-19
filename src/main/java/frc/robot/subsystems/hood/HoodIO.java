package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double Setpoint = 0.0;
    public double Velocity = 0.0;
    public double Accerleration = 0.0;
    public double Temp = 0.0;
    public double Voltage = 0.0;
    public double Current = 0.0;
    public double Position = HoodConstants.INITIAL_SETPOINT;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void changeSetpoint(double setpoint) {}
}
