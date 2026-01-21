package frc.robot.subsystems.floor;

import frc.robot.subsystems.floor.FloorConstants.RollerSetpoint;
import org.littletonrobotics.junction.AutoLog;

public interface FloorIO {

  @AutoLog
  public static class FloorIOInputs {
    public double setpoint = 0.0;
    public double velocity = 0.0;
    public double temp = 0.0;
    public double voltage = 0.0;
    public double current = 0.0;
    public double position = 0.0;
    public double acceleration = 0.0;
  }

  public default void updateInputs(FloorIOInputs inputs) {}

  public default void changeSetpoint(RollerSetpoint newSetpoint) {}

  public default void changeSetpoint(double setpoint) {}
}
