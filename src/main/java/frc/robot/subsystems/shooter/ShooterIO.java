package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public static double setpoint = 0.0;
    public static double velocity = 0.0;
    public static double position = 0.0;
    public static double acceleration = 0.0;

    public static double leaderMotorTemp = 0.0;
    public static double leaderMotorVoltage = 0.0;
    public static double leaderMotorCurrent = 0.0;

    public static double followerMotorTemp = 0.0;
    public static double followerMotorVoltage = 0.0;
    public static double followerMotorCurrent = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void changeSetpoint(double Setpoint) {}
}
